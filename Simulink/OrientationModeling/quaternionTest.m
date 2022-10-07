a = 15;
B = 20;
Y = 35;

% a = yaw
% B = pitch
% Y = roll

c_a = cos(a / 2);
s_a = sin(a / 2);
c_B = cos(B / 2);
s_B = sin(B / 2);
c_Y = cos(Y / 2);
s_Y = sin(Y / 2);

qw = c_Y * c_B * c_a + s_Y * s_B * s_a;
qi = s_Y * c_B * c_a - c_Y * s_B * s_a;
qj = c_Y * s_B * c_a + s_Y * c_B * s_a;
qk = c_Y * c_B * s_a - s_Y * s_B * c_a;

disp("(" + qw + "," + qi + "," + qj + "," + qk + ")");