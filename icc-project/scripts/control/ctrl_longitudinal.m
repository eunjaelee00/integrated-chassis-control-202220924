function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 (속도 추종 + ABS)
%
%   설계 개요
%   ---------
%   - 속도 추종 : cruise/decel transition 용 PI. 단, 본 plant 루프에는 구동
%                 actuator 가 없으므로(brake 만 인가 가능) Fx_total>0(가속)은
%                 실제로 작동하지 않고 Fx_total<0(제동)만 coordinator 가 사용.
%                 → 채점 시나리오에서는 속도 PI 가 사실상 dormant.
%   - ABS       : 핵심. runner 가 매 step ctrlState.wheelSlip(4x1) 에 직전 step
%                 의 휠 slip ratio(kappa, 제동 시 음수)를 캐시해 준다.
%                 휠별로 kappa 를 목표(-0.12)로 되돌리는 PI 를 돌려, brake 를
%                 "줄이는" 방향(brakeMod ≤ 0)의 명령을 만든다.
%                 coordinator 가 이 brakeMod 를 brake 토크에 더하고, 최종 루프가
%                 brk_scenario + brakeMod 를 [0,MAX]로 클립 → 잠긴 휠이 풀린다.
%
%   forceCmd.brakeMod (4x1) 는 starter 외 추가 필드이며 ctrl_coordinator 에서 읽음.

    %% ---- 튜닝 파라미터 ---------------------------------------------------
    Kp_v  = CTRL.LON.Kp;     Ki_v = CTRL.LON.Ki;        % 속도 PI
    m_est = 1600;            % 대략 차량 질량 [kg] (force 환산용; coordinator 와 일치)

    kappaTarget = -0.12;     % ABS 목표 slip ratio (제동, mu_peak 근처)
    Kp_abs = 4000;           % ABS 비례 게인 [Nm / slip]
    Ki_abs = 30000;          % ABS 적분 게인
    absIntMax = LIM.MAX_BRAKE_TRQ;     % ABS 적분 windup 한계
    %% --------------------------------------------------------------------

    if ~isfield(ctrlState, 'intError'); ctrlState.intError = 0; end
    if ~isfield(ctrlState, 'absInt');   ctrlState.absInt   = zeros(4,1); end
    if ~isfield(ctrlState, 'wheelSlip') || numel(ctrlState.wheelSlip) < 4
        ctrlState.wheelSlip = zeros(4,1);
    end

    %% (1) 속도 추종 PI (anti-windup)
    ev = vxRef - vx;                                   % [m/s]
    ctrlState.intError = ctrlState.intError + ev * dt;
    ctrlState.intError = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intError));
    a_cmd = Kp_v * ev + Ki_v * ctrlState.intError;     % 요구 가속도 [m/s^2]
    a_cmd = max(-LIM.MAX_AX, min(LIM.MAX_AX, a_cmd));
    Fx_total = m_est * a_cmd;                          % [N] (양수 가속/음수 제동)

    %% (2) ABS — 휠별 slip-regulating PI (release-only)
    kappa = ctrlState.wheelSlip(:);                    % 4x1, 제동 시 음수
    braking = (ax < -0.2);                             % 감속 중일 때만 ABS 활성
    brakeMod = zeros(4,1);
    if braking
        for i = 1:4
            % err<0 → 목표보다 더 잠김(과슬립) → 음의 brakeMod 로 release
            err = kappa(i) - kappaTarget;
            if err < 0
                ctrlState.absInt(i) = ctrlState.absInt(i) + err * dt;
                ctrlState.absInt(i) = max(-absIntMax, min(0, ctrlState.absInt(i)));
                brakeMod(i) = Kp_abs * err + Ki_abs * ctrlState.absInt(i);
            else
                % 충분히 회복되면 적분 천천히 해제 (재가압)
                ctrlState.absInt(i) = min(0, ctrlState.absInt(i) + 0.5 * err * dt);
                brakeMod(i) = 0;
            end
        end
        brakeMod = max(-LIM.MAX_BRAKE_TRQ, min(0, brakeMod));   % release 전용
    else
        ctrlState.absInt = zeros(4,1);                 % 비제동 시 리셋
    end

    %% (3) 출력
    forceCmd.Fx_total   = Fx_total;
    forceCmd.brakeRatio = double(Fx_total < 0);
    forceCmd.brakeMod   = brakeMod;                    % 4x1 (coordinator 가 사용)
end
