# Daily workflow — where files come from and where they go

## The short answer

**I write files directly into this folder.** They appear on your Desktop
automatically, with no upload step and no copy-paste. You do not create any of
them by hand.

What you *do* is run `hexsync` to mirror them into WSL, then build.

## The full picture

```
   Claude writes here                    You build here
   ─────────────────────                 ────────────────────
   C:\Users\Rushi Tangirala\             ~/hexapod_ws/
     OneDrive\Desktop\hexapod_scratch\
                                          
     src/          ──── hexsync ────>     src/        (mirror — DO NOT EDIT)
     tools/                               build/      (WSL only)
     README.md                            install/    (WSL only)
     WORKFLOW.md                          log/        (WSL only)
     .git/         <── git lives here
```

**Rule: edit only on the Windows side.** `hexsync` runs `rsync --delete`, so
anything you change inside `~/hexapod_ws/src/` is destroyed on the next sync.
That is deliberate — one source of truth, no divergence.

## Opening it in VS Code

Two workable setups. Pick one and stay with it.

### Option A — VS Code on Windows (simplest)

```
File > Open Folder > C:\Users\Rushi Tangirala\OneDrive\Desktop\hexapod_scratch
```

Then open a WSL terminal inside VS Code:

- `` Ctrl+` `` to open the terminal panel
- Click the **∨** next to the `+` and choose **Ubuntu (WSL)**
- Run `hexbuild` from there

You edit Windows files, you build in WSL, both in one window. Recommended.

### Option B — VS Code Remote-WSL

Install the **WSL** extension, press `F1`, run **WSL: Connect to WSL**, then
open `/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch`.

Better ROS/Python IntelliSense (the interpreter is the real Ubuntu one), but
file access over `/mnt/c` is slower. Note you still open the *Windows* path —
never `~/hexapod_ws`.

## The loop

```bash
# 1. I write / you edit files in hexapod_scratch
# 2. mirror + build + source, one command:
hexbuild
# 3. run
ros2 launch hexapod_description display.launch.py
```

## When to do a CLEAN rebuild

```bash
cd ~/hexapod_ws && rm -rf build install log && hexbuild
```

Do this whenever:

- a **new directory** appears in a package (CMake caches its configure-time view)
- you renamed or deleted a file and stale copies seem to linger
- a change "has no effect" for no visible reason

It costs a few seconds on this project. When in doubt, clean.

## Git

The repository lives on the **Windows side**, in this folder — not in
`~/hexapod_ws`, which is a disposable build mirror.

```bash
cd /mnt/c/Users/'Rushi Tangirala'/OneDrive/Desktop/hexapod_scratch
git init -b main
git add .
git commit -m "chore: scaffold workspace, tooling and robot description"
```

If git complains about ownership on a Windows path:

```bash
git config --global --add safe.directory \
  "/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch"
```

Set your identity once if you have not:

```bash
git config --global user.name  "Varun"
git config --global user.email "hexapod69420@gmail.com"
```
