function a = PN_guidance(r_rel, V_rel, V_m_vec, N)
    % 比例导引法

    omega = cross(r_rel, V_rel) / norm(r_rel)^2;

    % a = - N * norm(V_rel) * cross(r_rel, omega) / norm(r_rel);
    % a = N * cross(V_rel, omega);
    a = - N * norm(V_rel) * cross(V_m_vec, omega) / norm(V_m_vec);
end
