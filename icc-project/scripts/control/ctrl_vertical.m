function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC — semi-active skyhook (Karnopp on-off)
%
%   sprung mass 절대속도(zs_dot)와 댐퍼 상대속도(zs_dot-zu_dot)의 부호가 같으면
%   (=댐퍼가 body 운동을 줄이는 방향) cMax, 아니면 cMin.  semi-active 라 항상
%   안정(c>0). body bounce(1-2Hz)를 절대좌표 기준 억제.
%
%   주: 강한 stiffening 을 제동 구간까지 적용하면 B1 ABS slip 동특성이 바뀌어
%   absSlipRMS 마진이 얇아짐을 실측 확인 → 제동 안정성(B1 5점) 보호를 위해
%   고전 on-off(하한 cMin) 유지. 저차원 plant 에서는 passive 로 fallback.

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;

    dampingCmd = 1500*ones(4,1);   % passive fallback

    if isempty(suspState) || ~isstruct(suspState) ...
       || ~isfield(suspState,'zs_dot') || ~isfield(suspState,'zu_dot') ...
       || numel(suspState.zs_dot)<4 || numel(suspState.zu_dot)<4
        return;
    end

    zs_dot = suspState.zs_dot(:);
    zu_dot = suspState.zu_dot(:);
    v_rel  = zs_dot - zu_dot;

    for i = 1:4
        if zs_dot(i)*v_rel(i) > 0
            dampingCmd(i) = cMax;
        else
            dampingCmd(i) = cMin;
        end
    end
end
