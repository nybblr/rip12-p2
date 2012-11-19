For part 3 and 4.1, no special libraries or addons were used.

4.2 requires the open source spacenavd daemon, and should compile
without any other external dependencies.

3: See images of configurations for reference.

4.1: Path Optimization To recreate results, position L1 at -90.0 and
move L1 such that the arm is just left of the suitcase. For the goal,
move L1 such that the arm is just right of the suitcase. Then execute
with greedy and connect and toggle the smooth/tree-type option.

4.2: Tele-operation

Proper use of the Tele-operation code requires a 3dconnexxion space
navigator mouse and a Linux machine with spacenavd.

Set the start point to whatever you wish. Click the 'Keep running'
checkbox. Click "Run", and move the mouse in a desired direction. The
simulator will pause each time it reads one tick from the mouse.

If the goal point is placed inside or beyond an obstacle, strange
behavior may result. The implementation of the potential field
navigation was not complete enough to send, and the naive placeholder is
not well behaved..
