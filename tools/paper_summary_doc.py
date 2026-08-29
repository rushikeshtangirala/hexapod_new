from docx import Document
from docx.shared import Pt, Inches, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

doc=Document()
st=doc.styles["Normal"]; st.font.name="Calibri"; st.font.size=Pt(10)
st.paragraph_format.space_after=Pt(4); st.paragraph_format.line_spacing=1.06
for n,sz in (("Heading 1",12),("Heading 2",10.5)):
    s=doc.styles[n]; s.font.name="Cambria"; s.font.size=Pt(sz); s.font.bold=True
    s.font.color.rgb=RGBColor(0x1F,0x1F,0x1F)
    s.paragraph_format.space_before=Pt(7 if n=="Heading 1" else 5)
    s.paragraph_format.space_after=Pt(3)
for s in doc.sections:
    s.top_margin=s.bottom_margin=Inches(0.55); s.left_margin=s.right_margin=Inches(0.7)

GREY=RGBColor(0x55,0x55,0x55)
def p(t,size=10,bold=False,align=None,after=4,italic=False,color=None):
    q=doc.add_paragraph(); r=q.add_run(t); r.font.size=Pt(size); r.bold=bold; r.italic=italic
    if color: r.font.color.rgb=color
    if align is not None: q.alignment=align
    q.paragraph_format.space_after=Pt(after); return q
def h1(t): return doc.add_heading(t,level=1)
def h2(t): return doc.add_heading(t,level=2)
def b(t,size=10):
    q=doc.add_paragraph(style="List Bullet"); r=q.add_run(t); r.font.size=Pt(size)
    q.paragraph_format.space_after=Pt(1); q.paragraph_format.line_spacing=1.0; return q
def shade(c,f):
    tcPr=c._tc.get_or_add_tcPr(); sh=OxmlElement("w:shd")
    sh.set(qn("w:val"),"clear"); sh.set(qn("w:color"),"auto"); sh.set(qn("w:fill"),f); tcPr.append(sh)
def table(hd,rows,widths,fs=9):
    t=doc.add_table(rows=1,cols=len(hd)); t.style="Table Grid"; t.alignment=WD_TABLE_ALIGNMENT.CENTER
    for i,x in enumerate(hd):
        c=t.rows[0].cells[i]; c.text=""
        r=c.paragraphs[0].add_run(x); r.bold=True; r.font.size=Pt(fs)
        c.paragraphs[0].paragraph_format.space_after=Pt(0); shade(c,"E8E8E8")
    for row in rows:
        cells=t.add_row().cells
        for i,v in enumerate(row):
            cells[i].text=""
            r=cells[i].paragraphs[0].add_run(v); r.font.size=Pt(fs); r.bold=(i==0)
            cells[i].paragraphs[0].paragraph_format.space_after=Pt(0)
            cells[i].paragraphs[0].paragraph_format.line_spacing=1.0
    for row in t.rows:
        for i,c in enumerate(row.cells): c.width=Inches(widths[i])
    q=doc.add_paragraph(); q.paragraph_format.space_after=Pt(3)
    return t

C=WD_ALIGN_PARAGRAPH.CENTER
p("Research Paper Summary",15,True,C,after=2)
p("Development of a Bionic Hexapod Robot with Adaptive Gait and Clearance for "
  "Enhanced Agricultural Field Scouting",11,True,C,after=4)
p("Zhenghua Zhang, Weilong He, Fan Wu, Lina Quesada, Lirong Xiang  ·  "
  "Frontiers in Robotics and AI, 2024, 11:1426269", 9,False,C,after=2,color=GREY)
p("Application: agricultural field scouting and autonomous navigation",
  9,False,C,after=8,color=GREY,italic=True)

h1("1.  Problem Statement")
p("Traditional agricultural robots such as wheeled vehicles and drones have limitations in "
  "uneven, obstructed and unstructured agricultural environments. Wheeled robots can damage "
  "soil and plants and adapt poorly to terrain, while drones have limited ground-level sensing "
  "and are affected by wind and obstacles. The researchers therefore developed a six-legged "
  "robot capable of stable movement over complex terrain.")

h1("2.  Objectives")
for t in ["Develop a lightweight and flexible bionic hexapod robot.",
          "Enable movement over different terrains and slopes.",
          "Develop terrain-adaptive gait and adjustable ground clearance.",
          "Maintain robot stability using an IMU.",
          "Add LiDAR, stereo cameras and distance sensors for autonomous navigation and "
          "obstacle avoidance.",
          "Evaluate performance in realistic agricultural environments."]: b(t)

h1("3.  Robot Design")
p("The robot has six legs with three degrees of freedom each, giving eighteen degrees of "
  "freedom in total. This provides high mobility and allows individual leg positions to be "
  "adjusted for uneven terrain. The body is lightweight and modular, and the robot operates in "
  "two clearance modes: a low-clearance mode for efficient movement on relatively flat terrain, "
  "and a high-clearance mode for crossing taller obstacles. The standard version uses an IMU "
  "and force sensors; the advanced version adds LiDAR, stereo cameras and distance sensors.")
table(["Specification","Value","Specification","Value"],
 [["Mass","4.2 kg without batteries, 4.7 kg with two","Maximum speed","Approximately 1.2 m/s"],
  ["Payload","8 kg","Battery","Two 3S LiPo batteries"],
  ["Ground clearance","6 to 18 cm","Endurance","3 h standing, about 1 h at 0.5 m/s"]],
 [1.05,2.4,1.05,2.6])

h1("4.  Locomotion and Adaptive Gait")
p("The key contribution is a terrain-adaptive locomotion system: the robot changes its gait "
  "and leg clearance according to the terrain and obstacle height. The controller takes sensor "
  "information and computes joint angles, then walking speed, then direction, and finally the "
  "servo commands. This allows the robot to remain stable while crossing uneven ground instead "
  "of using one fixed walking pattern.")

h1("5.  Experimental Results")
p("The robot was tested on concrete, grass, rugged grass, slopes, uneven terrain, and terrain "
  "containing obstacles. It maintained good stability, with pitch fluctuations between "
  "approximately −11.5° and +8.6°, and operated successfully on slopes of up to 17°. Most "
  "importantly, the terrain-adaptive algorithm reduced energy consumption by approximately "
  "14.4% per obstacle crossed compared with traditional obstacle-avoidance methods.")

h1("6.  Conclusion")
p("The study demonstrates that a bionic hexapod with adaptive gait and variable clearance "
  "provides excellent terrain adaptability for agricultural applications. Its six-legged "
  "structure gives multiple support points, allowing it to remain stable over difficult "
  "terrain. The authors conclude that the robot has strong potential for precision "
  "agriculture, field scouting and autonomous agricultural operations, with future work on "
  "energy efficiency, durability and compatibility with different crops.")
p("Key takeaway: a hexapod can use its 18-DOF leg system, sensors and adaptive gait to "
  "automatically change how it walks and how high it lifts its legs, allowing stable and "
  "energy-efficient movement through difficult agricultural terrain.",bold=True,after=6)

doc.add_page_break()
h1("7.  Comparison with Our Project")
p("The paper is a useful benchmark because the machine is almost identical to ours: six legs "
  "with three degrees of freedom, the tripod gait as the starting point, kinematics presented "
  "for the leg, and simulation carried out before hardware. Every difference below is "
  "therefore a genuine engineering choice rather than a difference of scale.")
table(["","Zhang et al., 2024","Our project"],
 [["Topology","6 legs x 3 DOF, tripod gait as base","Same"],
  ["Mass","4.7 kg with batteries, 8 kg payload","3.39 kg"],
  ["Structure","Carbon fibre and CNC aluminium","3D printed PLA throughout"],
  ["Foot trajectory","Sinusoidal joint modulation; IK avoided at run time",
   "Closed-form analytic IK at 50 Hz"],
  ["Commanding","A gait mode is selected","A body velocity is commanded, stride derived"],
  ["Clearance","Adjustable 6 to 18 cm, terrain adaptive","Fixed 45 mm, already a gait parameter"],
  ["Simulation","SolidWorks into Simulink Simscape","AutoCAD into URDF, Gazebo, ros2_control"],
  ["Sensing","IMU and one force sensor per foot","Foot sensors modelled, currently disabled"],
  ["Validation","Field trials on four terrain types","498 offline assertions, IK exact to 3.5e-16 m"],
  ["Stage","Hardware built and field tested","Simulation complete, hardware next"]],
 [1.0,3.05,3.05])

h2("The main difference in approach")
p("The paper deliberately avoids inverse kinematics at run time, arguing that solving it at "
  "each step is computationally expensive, and drives the joints with sinusoidal modulation "
  "instead. That objection applies to numerical inverse kinematics, which iterates. Ours is "
  "closed form: a fixed number of trigonometric evaluations, no iteration, and the same cost "
  "on every call. It is inexpensive, and it buys what the sinusoidal method cannot offer. With "
  "sinusoidal modulation the operator selects a gait mode; with analytic inverse kinematics "
  "the operator commands a body velocity and the stride for every leg follows from geometry, "
  "which is why walking, strafing, turning and driving an arc are one piece of code. In short, "
  "their design trades commandability for computation and ours trades computation for "
  "commandability; a closed-form solution is what makes that trade affordable. Both are valid: "
  "theirs suits a battery-limited scouting robot where cost of transport is the headline metric.")

h2("Validation, and where we sit")
p("They validate empirically through field trials, which requires built hardware; we validate "
  "formally offline through automated assertions. The approaches are complementary, and ours "
  "caught a stance sign error that field testing would probably have dismissed as a robot "
  "needing tuning. They have a built robot with completed trials, while we have a complete "
  "simulation with verified mathematics and hardware as the next phase. Their sensing package "
  "is essentially our next-phase list, and their adjustable clearance is a natural extension "
  "of our formulation, since our step height is already a parameter of the swing equation.")

doc.save("Research_Paper_Summary.docx")
print("written")
