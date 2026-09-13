#!/usr/bin/env python3
"""
stem_detector.py : find plant stems in the forward camera.

=============================================================================
1. WHAT THIS NODE IS FOR
=============================================================================
Checkpoint 9 needs to answer one question sixty times a second: "where is the
nearest stem, relative to the robot?" This node answers it, in metres, in
base_link coordinates. Everything else here is in service of that.

It deliberately knows nothing about walking, gaits or Gazebo. It consumes a
sensor_msgs/Image and publishes a position. Point it at a real camera and it
behaves identically, which is the entire reason perception is its own package.

=============================================================================
2. WHY COLOUR, AND WHAT COLOUR SPACE
=============================================================================
The targets are green, the world is not. That makes colour the cheapest
sufficient discriminator, and cheap matters: this runs beside a physics
simulation on the same machine.

But NOT in RGB. In RGB, "green" is a diagonal region whose position moves with
brightness, so a threshold tuned in sunlight fails in shade and vice versa.
HSV separates what a thing IS from how brightly it is lit:

    H  hue          which colour            largely lighting invariant
    S  saturation   how pure the colour     low for grey, white, black
    V  value        how bright              varies enormously with lighting

So the hue window does the identifying, and the saturation and value floors
exist only to reject pixels where hue is meaningless. Hue of a nearly grey
pixel is numerical noise: a shadow on the ground can report any hue at all.
Rejecting S < 80 removes that entire failure class.

OpenCV packs H into 0..179, not 0..359, because the byte has to hold it. Green
sits near 60 in those units. Getting this wrong by a factor of two is the most
common first bug in any OpenCV colour project.

=============================================================================
3. WHY SHAPE AS WELL
=============================================================================
Colour alone finds the right pixels and the wrong objects. A green patch of
grass, a reflection, a distant green wall all pass a hue test. A stem has a
property none of those reliably have: it is much taller than it is wide.

So the contour filter requires a minimum area (rejects speckle) and a minimum
height to width ratio (rejects blobs and horizontal features). Two independent
criteria that a false positive must satisfy simultaneously is worth far more
than one tightened criterion, because the failure modes are uncorrelated.

=============================================================================
4. HOW RANGE IS RECOVERED FROM A SINGLE CAMERA
=============================================================================
One camera cannot measure depth. It can, however, measure depth to a point
KNOWN TO LIE ON A PLANE, and the base of a stem lies on the ground.

This is inverse perspective mapping and it is worth understanding, because it
is the one piece of real geometry in the project's perception:

  1. A pixel (u, v) corresponds to a ray leaving the camera centre. In the
     optical frame that ray is ((u - cx)/fx, (v - cy)/fy, 1), straight from
     the pinhole model. fx, fy, cx, cy come from /camera/camera_info, so
     changing the camera resolution needs no code change here.

  2. Rotate that ray from the optical frame into base_link. Two rotations: the
     REP 145 to REP 103 axis relabelling, then the camera's mount pitch.

  3. The camera sits at a known height h above the ground. Walk along the ray
     until it reaches the ground plane. That scalar t is fixed by the vertical
     component alone, and substituting it back gives x and y.

The accuracy is limited by h, which is why the assumption is stated loudly
below rather than buried. Everything else is exact.

ALTERNATIVE, AND WHY IT IS NOT USED. Apparent height: a 0.30 m stem covering
N pixels is at range f * 0.30 / N. Simpler, but it needs the whole stem in
frame, and it fails exactly when the robot is close and the stem fills the
image, which is the moment the range matters most.

=============================================================================
5. TOPICS
=============================================================================
Subscribes
    /camera/image_raw     sensor_msgs/Image
    /camera/camera_info   sensor_msgs/CameraInfo    intrinsics, read once

Publishes
    /stem/detected        std_msgs/Bool             every frame
    /stem/bearing         std_msgs/Float32          radians, + is left
    /stem/position        geometry_msgs/PointStamped  base_link, metres
    /stem/debug_image     sensor_msgs/Image         annotated, for rqt

bearing is published whenever anything is seen. position is published only
when the stem's BASE is inside the image, because that is the pixel the ground
plane solution needs. An approach controller can steer on bearing alone and
only needs position for the stop condition, so this split matches how the
information is actually used.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32


class StemDetector(Node):

    def __init__(self) -> None:
        super().__init__("stem_detector")

        # ------------------------------------------------------------------
        # Colour window. Declared as parameters, not constants, because the
        # right values depend on lighting and are the first thing anyone
        # tunes. Change them live with:
        #   ros2 param set /stem_detector h_min 35
        # ------------------------------------------------------------------
        self.declare_parameter("h_min", 40)
        self.declare_parameter("h_max", 80)
        self.declare_parameter("s_min", 80)
        self.declare_parameter("v_min", 40)

        # Shape gate.
        self.declare_parameter("min_area_px", 40.0)
        self.declare_parameter("min_aspect", 1.5)

        # ------------------------------------------------------------------
        # GEOMETRY THAT MUST MATCH hexapod.sensors.xacro AND THE STANCE.
        #
        # These two numbers are duplicated from the model, and duplicated
        # numbers drift. That has already cost this project real time, so it
        # is stated plainly rather than hidden:
        #
        #   camera_pitch  must equal cam_pitch in hexapod.sensors.xacro
        #   camera_height must equal the camera's height above the GROUND,
        #                 which is the body's stance height plus cam_z
        #
        # The principled fix is a tf2 lookup of base_link to
        # camera_optical_link, which removes the first duplication entirely.
        # It does not remove the second: TF knows where the camera is on the
        # robot, not how high the robot is standing. That still comes from
        # the stance, or from odometry once the robot walks freely.
        #
        # A wrong camera_height does not break detection or bearing. It
        # scales range linearly, so the robot still steers correctly and
        # simply stops at the wrong distance.
        # ------------------------------------------------------------------
        self.declare_parameter("camera_pitch", 25.0 * math.pi / 180.0)
        # 0.132 m, and the same number in both configurations, which is a
        # useful coincidence rather than a fudge. Anchored, it is
        # fix_base_height. Standing on its own feet, it is the stance depth
        # 0.120 plus the foot sphere radius 0.012. The camera sits at cam_z
        # of zero, so it is at base_link height either way.
        self.declare_parameter("camera_height", 0.132)
        self.declare_parameter("camera_x", 0.15635)

        self.declare_parameter("publish_debug", True)

        self.bridge = CvBridge()
        self.fx: float | None = None
        self.fy: float | None = None
        self.cx: float | None = None
        self.cy: float | None = None

        self.pub_detected = self.create_publisher(Bool, "/stem/detected", 10)
        self.pub_bearing = self.create_publisher(Float32, "/stem/bearing", 10)
        self.pub_position = self.create_publisher(
            PointStamped, "/stem/position", 10)
        self.pub_debug = self.create_publisher(Image, "/stem/debug_image", 2)

        # Intrinsics arrive once and never change. Reading them from the topic
        # rather than hard-coding means the camera resolution in the xacro can
        # be halved for performance without touching this file.
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_camera_info, 10)

        # Queue depth 1, deliberately. If detection falls behind the camera we
        # want the NEWEST frame, not a backlog of stale ones. A controller
        # acting on a 500 ms old bearing oscillates.
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 1)

        self.get_logger().info("stem_detector up, waiting for camera_info")

    # ----------------------------------------------------------------------
    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.fx is not None:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f"intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f} "
            f"({msg.width}x{msg.height})")

    # ----------------------------------------------------------------------
    def p(self, name: str):
        return self.get_parameter(name).value

    # ----------------------------------------------------------------------
    def ground_point(self, u: float, v: float) -> tuple[float, float] | None:
        """
        Where does the ray through pixel (u, v) meet the ground?

        Returns (x, y) in base_link metres, x forward and y left, or None if
        the ray does not point downward and therefore never reaches the
        ground. See section 4 of the module docstring.
        """
        if self.fx is None:
            return None

        # 1. Ray in the optical frame: x right, y down, z forward.
        rx = (u - self.cx) / self.fx
        ry = (v - self.cy) / self.fy
        rz = 1.0

        # 2a. Optical to camera_link. This is the inverse of the rpy
        #     (-pi/2, 0, -pi/2) in the xacro, and reduces to an axis swap:
        #     optical z forward becomes camera x forward, optical x right
        #     becomes camera minus y, optical y down becomes camera minus z.
        cxv, cyv, czv = rz, -rx, -ry

        # 2b. camera_link to base_link: rotate by the mount pitch about +Y.
        th = float(self.p("camera_pitch"))
        bx = math.cos(th) * cxv + math.sin(th) * czv
        by = cyv
        bz = -math.sin(th) * cxv + math.cos(th) * czv

        # 3. March to the ground. The camera is at height h, so we need the
        #    ray to descend. A ray level with or above the horizon never
        #    intersects, which is the physical meaning of this guard, not a
        #    numerical nicety.
        if bz > -1e-6:
            return None

        h = float(self.p("camera_height"))
        t = h / (-bz)
        return (float(self.p("camera_x")) + t * bx, t * by)

    # ----------------------------------------------------------------------
    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h_img, w_img = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([self.p("h_min"), self.p("s_min"), self.p("v_min")],
                     dtype=np.uint8),
            np.array([self.p("h_max"), 255, 255], dtype=np.uint8),
        )

        # OPEN then CLOSE, in that order, and the order matters.
        # Open (erode then dilate) deletes anything thinner than the kernel:
        # single-pixel colour noise vanishes. Close (dilate then erode) fills
        # small holes: a stem split in two by a highlight becomes one contour
        # again. Doing them the other way round would first weld the noise
        # into the target and then be unable to separate them.
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < float(self.p("min_area_px")):
                continue
            x, y, w, hh = cv2.boundingRect(c)
            if w <= 0:
                continue
            if hh / float(w) < float(self.p("min_aspect")):
                continue
            # Largest qualifying blob wins. With several stems in frame that
            # is the nearest one, since apparent size falls with range. That
            # is the behaviour the approach controller wants and it comes out
            # of the geometry for free.
            if area > best_area:
                best_area = area
                best = (x, y, w, hh)

        detected = best is not None
        self.pub_detected.publish(Bool(data=detected))

        # ------------------------------------------------------------------
        # DEBUG OVERLAY.
        #
        # The mask is tinted onto the image, not just the winning box. A box
        # alone tells you WHAT was chosen and nothing about WHY: if the
        # threshold is catching something unexpected, the box lands in a
        # strange place and the picture gives you no way to tell whether the
        # colour stage or the shape stage is at fault. Tinting every surviving
        # pixel magenta makes the colour stage visible directly, so one glance
        # separates "the threshold caught the wrong thing" from "the threshold
        # was right and the contour logic picked badly".
        #
        # The red cross marks the exact pixel handed to the ground-plane
        # solver. That single pixel is the entire input to the range estimate,
        # so seeing where it actually lands is the fastest way to understand a
        # wrong distance.
        # ------------------------------------------------------------------
        debug = bool(self.p("publish_debug"))
        annotated = frame
        if debug:
            annotated = frame.copy()
            sel = mask > 0
            if sel.any():
                annotated[sel] = np.clip(
                    0.4 * annotated[sel].astype(np.float32)
                    + np.array([150.0, 0.0, 150.0], dtype=np.float32),
                    0, 255).astype(np.uint8)

        if detected:
            x, y, w, hh = best
            u_centre = x + w / 2.0
            v_base = y + hh          # bottom of the box: where it meets soil

            bearing = math.atan2(u_centre - self.cx, self.fx) \
                if self.fx else 0.0
            # atan2 above is measured with +u to the right of the image, and
            # +y in base_link is LEFT, hence the negation. Getting this sign
            # wrong makes the robot steer away from every target it sees.
            self.pub_bearing.publish(Float32(data=float(-bearing)))

            # The base pixel is only meaningful if the stem's foot is inside
            # the frame. Touching the bottom row means the true base is below
            # the image and the ground solution would be badly wrong, so no
            # position is published and the controller keeps steering on
            # bearing alone.
            base_visible = v_base < (h_img - 2)
            gp = self.ground_point(u_centre, v_base) if base_visible else None

            if gp is not None:
                pt = PointStamped()
                pt.header.stamp = msg.header.stamp
                pt.header.frame_id = "base_link"
                pt.point.x, pt.point.y, pt.point.z = gp[0], gp[1], 0.0
                self.pub_position.publish(pt)

            # Throttled, so it is readable rather than a wall of text at
            # camera rate. These are the raw numbers behind the published
            # position: box in pixels, the base pixel used, and the result.
            # If the position is wrong, the fault is visible in exactly one
            # of these three and the log says which.
            self.get_logger().info(
                f"box x={x} y={y} w={w} h={hh}  "
                f"base_px=({u_centre:.0f},{v_base:.0f})  "
                f"n_contours={len(contours)}  "
                f"-> {('x=%.3f y=%.3f' % gp) if gp else 'no ground solution'}",
                throttle_duration_sec=2.0)

            if debug:
                cv2.rectangle(annotated, (x, y), (x + w, y + hh),
                              (0, 255, 255), 2)
                cv2.drawMarker(annotated, (int(u_centre), int(v_base)),
                               (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
                label = (f"{gp[0]:.2f}m  {math.degrees(-bearing):+.1f}deg"
                         if gp else f"{math.degrees(-bearing):+.1f}deg  "
                                    f"(base out of frame)")
                cv2.putText(annotated, label, (x, max(14, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if debug:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = msg.header
            self.pub_debug.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StemDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
