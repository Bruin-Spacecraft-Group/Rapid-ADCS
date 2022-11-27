function q1q2 = quatmult(q1, q2)
% Multiplication of quaternions q and p (4-vector inputs)

    if (length(q1)==3) || (length(q2)==3)
        if length(q1) == 3
            q1o = 0; q1v = q1(1:3);
        else
            q1o = q1(1); q1v = q1(2:4);
        end
        if length(q2) == 3
            q2o = 0; q2v = q2(1:3);
        else
            q2o = q2(1); q2v = q2(2:4);
        end
    elseif (length(q1)==4) && (length(q2)==4)
            q1o = q1(1); q1v = q1(2:4);
            q2o = q2(1); q2v = q2(2:4);
    else
        error('quat dim error')
    end
    q1q2o = q1o*q2o - dot(q1v,q2v);
    q1q2v = q1o*q2v + q2o*q1v + cross(q1v,q2v);
    q1q2 = q1q2o*[1; 0; 0; 0] + [0 0 0; eye(3)]*q1q2v;
end