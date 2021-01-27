function [P] = get_P(T,Qc,Cqw_i,a_skew_estimated,wskew_estimated,P)

    Bd = -Cqw_i;
    ax = a_skew_estimated;
    term = Bd*ax;
    wx = wskew_estimated;
    
    s_na  = Qc(1:3,1:3);
    s_nba = Qc(4:6,4:6);
    s_nw  = Qc(7:9,7:9);
    s_nbw = Qc(10:12,10:12);
    
    int_F = -eye(3)*T^2/2+T^3/6*wx-T^4/24*wx^2; %done
    int_B = term*(-eye(3)*T^4/24+T^5/120*wx-T^6/720*wx^2); %done
    int_D = -term*(eye(3)*T^3/6-T^4/24*wx+T^5/120*wx^2); %done
    
    int_AAt = term*(eye(3)*T^5/20+T^7/504*wx^2)*term'; %done
    int_BBt = term*(eye(3)*T^7/252+T^9/8640*wx^2)*term'; %done
    int_CCt = term*(eye(3)*T^3/3+T^5/60*wx^2)*term'; %done
    int_DDt = term*(eye(3)*T^5/20+T^7/504*wx^2)*term'; %done
    int_EEt = (eye(3)*T); %done
    int_FFt = (eye(3)*T^3/3+T^5/60*wx^2); %done
    
    int_EAt = (eye(3)*T^3/6-T^4/12*wx+T^5/40*wx^2)*term'; %done
    int_FBt = (eye(3)*T^5/30-T^6/144*wx+11*T^7/5040*wx^2)*term'; %done
    int_ECt = (eye(3)*T^2/2-T^3/6*wx+T^4/24*wx^2)*term'; %done 
    int_FDt = (eye(3)*T^4/8-T^5/60*wx+T^6/144*wx^2)*term'; %done
    int_ACt = term*(eye(3)*T^4/8+T^5/60*wx+T^6/144*wx^2)*term'; %done
    int_BDt = term*(eye(3)*T^6/72+T^7/1008*wx+T^8/1920*wx^2)*term'; %done
    int_AEt = term*(eye(3)*T^3/6+T^4/12*wx+T^5/40*wx^2); %done
    int_BFt = term*(eye(3)*T^5/30+T^6/144*wx+11*T^7/5040*wx^2); %done
    int_CAt = term*(eye(3)*T^4/8-T^5/60*wx+T^6/144*wx^2)*term'; %done
    int_DBt = term*(eye(3)*T^6/72-T^7/1008*wx+T^8/1920*wx^2)*term'; %done
    int_CEt = term*(eye(3)*T^2/2+T^3/6*wx+T^4/24*wx^2); %done
    int_DFt = term*(eye(3)*T^4/8+T^5/60*wx+T^6/144*wx^2); %done
    
    Q11 = (T^3/3)*s_na*(Bd*Bd')+s_nw*int_AAt+s_nbw*int_BBt+(T^5/20)*s_nba*(Bd*Bd');
    Q12 = (T^2/2)*s_na*(Bd*Bd')+s_nw*int_ACt+s_nbw*int_BDt+(T^4/8)*s_nba*(Bd*Bd');
    Q13 = s_nw*int_AEt+s_nbw*int_BFt;
    Q14 = s_nbw*int_B;
    Q15 = (T^3/6)*s_nba*Bd;
    Q16 = zeros(3,13);
    
    Q21 = (T^2/2)*s_na*(Bd*Bd')+s_nw*int_CAt+s_nbw*int_DBt+(T^4/8)*s_nba*(Bd*Bd');
    Q22 = (T)*s_na*(Bd*Bd')+s_nw*int_CCt+s_nbw*int_DDt+(T^3/3)*s_nba*(Bd*Bd');
    Q23 = s_nw*int_CEt+s_nbw*int_DFt;
    Q24 = s_nbw*int_D;
    Q25 = (T^2/2)*s_nba*Bd;
    Q26 = zeros(3,13);
    
    Q31 = s_nw*int_EAt+s_nbw*int_FBt;
    Q32 = s_nw*int_ECt+s_nbw*int_FDt;
    Q33 = s_nw*int_EEt+s_nbw*int_FFt;
    Q34 = s_nbw*int_F;
    Q35 = zeros(3);
    Q36 = zeros(3,13);
    
    Q41 = s_nbw*int_B';
    Q42 = s_nbw*int_D';
    Q43 = s_nbw*int_F';
    Q44 = s_nbw*T;
    Q45 = zeros(3);
    Q46 = zeros(3,13);
    
    Q51 = s_nba*Bd'*(T^3/6);
    Q52 = s_nba*Bd'*(T^2/2);
    Q53 = zeros(3);
    Q54 = zeros(3);
    Q55 = s_nba*T;
    Q56 = zeros(3,13);
    
    Q61 = zeros(13,3);
    Q62 = zeros(13,3);
    Q63 = zeros(13,3);
    Q64 = zeros(13,3);
    Q65 = zeros(13,3);
    Q66 = zeros(13,13);
    
    Qd = [Q11,Q12,Q13,Q14,Q15,Q16;...
          Q21,Q22,Q23,Q24,Q25,Q26;...
          Q31,Q32,Q33,Q34,Q35,Q36;...
          Q41,Q42,Q43,Q44,Q45,Q46;...
          Q51,Q52,Q53,Q54,Q55,Q56;...
          Q61,Q62,Q63,Q64,Q65,Q66];
    
    A = term*((T^2/2)-(T^3/6)*wx+(T^4/24)*wx^2);
    B = term*(-(T^3/6)+(T^4/24)*wx-(T^5/120)*wx^2);
    C = term*(T-(T^2/2)*wx+(T^3/6)*wx^2);
    D = -A;
    E = eye(3)-T*wx+(T^2/2)*wx^2;
    F = -T+(T^2/2)*wx-(T^3/6)*wx^2;
    
    Fd = [eye(3),T*eye(3),A,B,Bd*(T^2/2),zeros(3,13);...
          zeros(3),eye(3),C,D,Bd*T,zeros(3,13);...
          zeros(3),zeros(3),E,F,zeros(3),zeros(3,13);...
          zeros(3),zeros(3),zeros(3),eye(3),zeros(3),zeros(3,13);...
          zeros(3),zeros(3),zeros(3),zeros(3),eye(3),zeros(3,13);...
          zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3),eye(13)];
     
    P = Fd*P*Fd'+Qd;
end

