% IMPORTED FROM SOLIDWORKS
T_sys1 = [
0.0255158,0.00322292,-0.00004932;
0.00322292,0.02570486,0.00005349;
-0.00004932,0.00005349,0.00961748
];

% Current coordinate system by Structures Subteam
coord_sys1 = [
0,-1,0;
0,0,1;
-1,0,0
];

T_out = fcn(T_sys1,coord_sys1);
format shortE;
disp(T_out);
fprintf('----------------------\nTo copy into Simulink:\n[');
for a = 1:3
    for b = 1:3
        fprintf('%.5e', T_out(a,b));
        if(a ~= 3 || b ~= 3)
            fprintf(',');
        end
    end
end
fprintf(']\n');

% transform tensor from given rotated cartesian coordinate system by unit vectors to standard cartesian 
function T_sys2 = fcn(T_sys1,coord_sys1)

% normalize coordinate systems
coord_sys1 = coord_sys1 / norm(coord_sys1);

c2 = [1,0,0;0,1,0;0,0,1];
c1 = coord_sys1;

trans_mat = [
dot(c2(1,:),c1(1,:)), dot(c2(1,:),c1(2,:)), dot(c2(1,:),c1(3,:));
dot(c2(2,:),c1(1,:)), dot(c2(2,:),c1(2,:)), dot(c2(2,:),c1(3,:));
dot(c2(3,:),c1(1,:)), dot(c2(3,:),c1(2,:)), dot(c2(3,:),c1(3,:));
];

T_sys2 = trans_mat * T_sys1;
T_sys2 = T_sys2 * trans_mat;
end