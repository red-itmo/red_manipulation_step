// ** Additional **
function R = Rz(psi)
    R = [
         cos(psi), sin(psi), 0;
        -sin(psi), cos(psi), 0;
         0,        0,        1;
    ];
endfunction
function R = Ry(theta)
    R = [
        cos(theta), 0, -sin(theta);
        0,        1,  0;
        sin(theta), 0,  cos(theta);
    ];
endfunction

function v_n = normalize(v)
    v_n = v/norm(v);
endfunction
