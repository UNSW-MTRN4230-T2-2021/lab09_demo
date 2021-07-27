
launch with demo.launch
then run node

moves, then spawns box, picks up and moves and drops off at new location

has awefully annoying warning about missing joint, this is because the joint is actuated by a plugin rather than moveit, but it is aware of the joint and therefore complains. You can ignore the warning and it performs fine.

if you find a fix please open a PR


