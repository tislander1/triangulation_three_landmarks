# triangulation_three_landmarks
Triangulating a position and heading using angles to three known landmarks.

- This code triangulates the position and heading of a robot from three landmarks which have known position.
- Note that Angles increase counterclockwise from above, just like standard math.  Forward = 0.
- This code and its notation is mostly from "A comprehensive study of three object triangulation" by Charles Cohen and Frank Koss, University of Michigan.  I coded the section 3.4, "Geometric Circle Intersection", although I needed to fill in the blanks a little bit. This code is based on 2 intersecting circles.  The first circle passes through beacon 1, beacon 2, and the robot (or human, etc), while the second circle passes through beacon 2, beacon 3, and the robot.  This geometric construction allows a simple solution to the position of the robot.  If the angles input are measured from the front of the robot, it also gives the heading.
