function a = PN_guidance(r, v, N)
    % proportional_guidance_3d - 三维比例导引法实现
    % 输入：
    %   r - 相对位置向量 (3×1)
    %   v - 相对速度向量 (3×1)
    %   N - 导引常数 (标量)
    % 输出:
    %   a - 惯性坐标系下的加速度
    omega = cross(r, v) / norm(r)^2;

    a = N * cross(v, omega);
end
