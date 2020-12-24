# Red object tracker & shape detector combined into one project for SSI AI Robotics course 2020

Webcam takes in live video feed & tracks a red object in view, traces the path where it moves, and if the path forms a convex quadrilateral somewhere, it gets highlighted in green.
I accomplished this by making 2 layers: one for the video feed, one for the tracked path drawing. The red object on the video feed layer is tracked and the trail is drawn on the second layer, and the shape detector looks solely at the second layer to find a convex quadrilateral.
