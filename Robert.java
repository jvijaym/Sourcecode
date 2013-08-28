import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.text.CharacterIterator;
import java.text.StringCharacterIterator;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
* This program moves a squad of robots around a rectangular plateau.
* The plateau is divided up into a grid of unit squares to simplify navigation.
*
* A robot's position and direction are represented by x and y coordinates
* and a letter representing one of the four cardinal compass points.
* e.g. "0 0 N" means the Robot is in the bottom left corner facing North.
*
* The square directly North of (x, y) is (x, y+1).
*
* A robot's movement are controlled by a sequence of letters. The possible letters
* are 'L', 'R' and 'M'. 'L' and 'R' makes the Robot spin 90 degrees left or right
* respectively without moving from its current spot. 'M' means move forward
* one grid point and maintain the same heading.
*
* Input is read from stdin and output is written to stdout.
*
* INPUT
* The first line is the x,y coordinates of top-right of the plateau boundary.
* e.g. "5 5" . (The lower left is at 0,0)
* The 2Nth line is the initial position/direction of the Nth Robot. e.g "1 2 N"
* The 2N+1th line is the movement commands for the Nth Robot. e.g "LMLMLMLMM"
* OUTPUT
* The Nth line is the final position and direction of the Nth Robot. e.g. "1 3 N"
*
* Each Robot will be run sequentially, which means that the second Robot
* won't start to move until the first one has finished moving.
*
* UNEXECUTABLE INSTRUCTIONS
* If a Robot is started in an invalid position then it will remain there.
* If a Robot is instructed to move from a valid position to an invalid position
* then it will remain in the valid position and not execute any more instruction.s
*
* MALFORMED INSTRUCTIONS
* If instructions are not formated exactly as described above then this program's
* behaviour is not defined.
*/

public class Robot {

/***********************************************************************
* Class to describe a Robot state
*/
static class State {
int _x, _y; // Robot position
int _theta;	// The angle of the direction the Robot is heading in units of PI/2 (0 points east)

// Copy constructor required for deep copies
State (State state) {
_x = state._x;
_y = state._y;
_theta = state._theta;
}

// Initialise Robot from a space separated string like "1 2 N"
State(String description) {
String[] parts = description.split(" ");
_x = Integer.parseInt(parts[0]);
_y = Integer.parseInt(parts[1]);
_theta = _codeAngleMap.get(parts[2].charAt(0));
}

// Convert state back to a space separated string
String asString() {
return "" + _x + " " + _y + " " + _angleCodeMap.get(_theta);
}

// Apply a transform to a Robot state (see definition of Xform)
// while keeping _theta within the range 0..3
void xform(Xform m) {
_theta = (_theta + m._dtheta) % 4;
if (_theta < 0)
_theta += 4;
_x += Math.round(Math.cos((double)_theta * Math.PI/2.0) * (double)m._distance);
_y += Math.round(Math.sin((double)_theta * Math.PI/2.0) * (double)m._distance);
}
};

// Class that describes a state transform.
static class Xform {
int _distance; // Distance to move
int _dtheta;	// Angle to rotate counter-clockwise in units of PI/2
Xform(int distance, int dtheta) {
_distance = distance;
_dtheta = dtheta;
}
};

/***********************************************************************
* Some mappings between the input/output encodings and the internal state
* and state transform representations described above.
*/
static final Map<Character, Integer> _codeAngleMap = new HashMap<Character, Integer>() {{
put('E', 0);
put('N', 1);
put('W', 2);
put('S', 3);
}};
static final Map<Integer, Character> _angleCodeMap = new HashMap<Integer, Character>() {{
for (Character code: _codeAngleMap.keySet())
put(_codeAngleMap.get(code), code);
}};
static final Map<Character, Xform> _codeXformMap = new HashMap<Character, Xform>() {{
put('M', new Xform(1, 0));
put('L', new Xform(0, 1));
put('R', new Xform(0, -1));
}};

/***********************************************************************
* The following state and functions describe a single Robot.
*/
State _state;

Robot(String description) {
_state = new State(description);
}

/* Process a single movement code.
* Move the Robot if the movement code is valid
* @return true for valid, false for invalid moves.
*/
boolean performInstruction(char code) {
State newState = new State(_state);
newState.xform(_codeXformMap.get(code));
boolean valid = isValidState(newState);
if (valid)
_state = newState;
return valid;
}

/* Process a list of movement codes and move the Robot accordingly.
* Stop when on the last valid code is reached.
* If the initial state is invalid then do nothing. This is the only case
* that leaves the Robot in an invalid state.
*/
void processInstructionList(String instructionList) {
if (isValidState(_state)) {
CharacterIterator it = new StringCharacterIterator(instructionList);
for (char code=it.first(); code != CharacterIterator.DONE; code=it.next()) {
if (!performInstruction(code))
break;
}
}
}

// @return state of Robot e.g. "1 2 N"
String getState() {
return _state.asString();
}

/***********************************************************************
* The following variables and methods describe the Robot's environment which
* comprises a plateau boundary and previous Robots.
*/
static public class Boundary {
private int _x0 = 0, _y0 = 0; // Coordinates of bottom left of boundary
private int _x1 = 0, _y1 = 0; // Coordinates of top right of boundary
public Boundary(String description) {
String[] parts = description.split(" ");
_x1 = Integer.parseInt(parts[0]);
_y1 = Integer.parseInt(parts[1]);
}
public boolean contains(int x, int y) {
return(_x0 <= x && x <= _x1 &&
_y0 <= y && y <= _y1);
}
};

static Boundary _boundary;
static List<Robot> _completedRobots = new ArrayList<Robot>();

static boolean isValidState(State state) {
boolean valid = true;
if (!_boundary.contains(state._x, state._y))
valid = false;
for (Robot Robot: _completedRobots) {
if (Robot._state._x == state._x && Robot._state._y == state._y) {
valid = false;
break;
}
}
return valid;
}

/***********************************************************************
* Process a stream of commands and respond with a stream of status strings.
* The input and output formats are specified in the INPUT and OUTPUT section
* of the comments at the start of this source file
*/
static void processCommandStream(BufferedReader input, BufferedWriter output) throws IOException {
String line;
if ((line = input.readLine()) != null)
_boundary = new Boundary(line);
while (line != null) {
if ((line = input.readLine()) != null) {
Robot Robot = new Robot(line);
if ((line = input.readLine()) != null) {
Robot.processInstructionList(line);
}
_completedRobots.add(Robot);
output.write(Robot.getState() + '\n') ;
output.flush();
}
}
}

/***********************************************************************
* Program entry point
* Processes a stream of Robot commands as described in processCommandStream()
* Reads input from stdin and writes output to stdout
*/
public static void main(String[] args) throws IOException {
BufferedReader input = new BufferedReader(new InputStreamReader(System.in));
BufferedWriter output = new BufferedWriter(new OutputStreamWriter(System.out));
try {
processCommandStream(input, output);
} finally {
input.close();
output.close();
}
}
}
