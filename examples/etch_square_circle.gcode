( Etch test: 20 x 20 mm square with a circle inside, 0.2 mm deep. )
( CAM-style coordinates: X0 Y0 = the square's lower-left corner, Z0 = copper surface. )
( Open with Import CAM... after setting the board origin and surface Z. )
( Circle: centre X10 Y10, radius 8 mm, so it sits 2 mm inside the square. )
G21
G90
M3 S1000
G0 Z2.0
( square )
G0 X0.0 Y0.0
G1 Z-0.2 F50.0
G1 X20.0 Y0.0 F100.0
G1 X20.0 Y20.0
G1 X0.0 Y20.0
G1 X0.0 Y0.0
G0 Z2.0
( circle )
G0 X18.0 Y10.0
G1 Z-0.2 F50.0
G2 X18.0 Y10.0 I-8.0 J0.0 F100.0
G0 Z2.0
M5
