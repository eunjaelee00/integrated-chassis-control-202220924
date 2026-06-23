function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation — 가중최소자승(WLS) + 마찰원 제한
%
%   제어수요 v_dem=[Mz_dem; Fx_dem] 를 4륜 brake T=[FL;FR;RL;RR] 로 WLS 할당:
%       min ||W^{1/2}(T-T0)||^2  s.t.  B·T = v_dem
%       해: T = T0 + Wi·B'·(B·Wi·B')^{-1}·(v_dem - B·T0)
%   - B(1,:): 차동 brake→yaw moment 유효도,  B(2,:): 4륜 brake→종력 유효도
%   - W: 후축 비용↑(후축 보호, 전축 접지 우선)
%   이후 (a) ABS release(lonCmd.brakeMod) 가산, (b) 선회 시 마찰원으로 휠별 제한.
%
%   단순 split 대비 동일 net Mz/Fx 를 내되 휠 부하를 최적 분산(가점 §5.3).
%   수요=0(직진제동 B1) 시 WLS·마찰원 모두 비활성 → ABS 동특성 보존.

    rw  = getf(VEH,'rw',0.31);
    htf = getf(VEH,'track_f',1.55)/2;
    htr = getf(VEH,'track_r',1.55)/2;
    m   = getf(VEH,'mass',1600);  g = 9.81;

    Mz_dem = latCmd.yawMoment;
    Fx_dem = 0;
    if isfield(lonCmd,'Fx_total') && lonCmd.Fx_total < 0
        Fx_dem = lonCmd.Fx_total;
    end
    v_dem = [Mz_dem; Fx_dem];

    % 유효도 행렬 (T>0 = 제동)
    B = [ htf/rw, -htf/rw,  htr/rw, -htr/rw;     % → Mz (좌측 brake = +CCW)
         -1/rw,   -1/rw,   -1/rw,   -1/rw  ];    % → Fx
    W  = diag([1.0, 1.0, 1.6, 1.6]);             % 후축 비용↑
    Wi = diag([1.0, 1.0, 1/1.6, 1/1.6]);

    if norm(v_dem) < 1e-6
        T = zeros(4,1);                          % 수요 0 → 정확히 0 (ABS 보존)
    else
        M = B*Wi*B';
        T = Wi*B'*(M \ v_dem);                   % WLS 최소노름 해 (T0=0)
    end

    % ABS release 가산 (음수)
    if isfield(lonCmd,'brakeMod') && numel(lonCmd.brakeMod) >= 4
        T = T + lonCmd.brakeMod(:);
    end

    % 마찰원 제한 — 선회(ESC 작동) 시에만, 직진제동 보호
    if abs(Mz_dem) > 1
        mu = 1.0; if isfield(CTRL,'COORD')&&isfield(CTRL.COORD,'mu'); mu=CTRL.COORD.mu; end
        Fz = 0.25*m*g;                           % 정적 근사 (per wheel)
        for i = 1:4
            fx = abs(T(i))/rw;                   % 종력 (횡력은 보수적으로 미반영)
            cap = mu*Fz;
            if fx > cap && abs(T(i)) > 1e-6
                T(i) = sign(T(i))*cap*rw;        % 마찰원 밖 → 종력 축소
            end
        end
    end

    T = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, T));
    steer = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    actuatorCmd.steerAngle   = steer;
    actuatorCmd.brakeTorque  = T;
    actuatorCmd.dampingCoeff = verCmd(:);
end

function v=getf(s,f,d); if isfield(s,f)&&~isempty(s.(f)); v=s.(f); else; v=d; end; end
