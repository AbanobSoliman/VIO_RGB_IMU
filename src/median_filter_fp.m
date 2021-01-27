function [Qvw_fp] = median_filter_fp(qvw_fp)

q0 = median(qvw_fp(:,1));
q1 = median(qvw_fp(:,2));
q2 = median(qvw_fp(:,3));
q3 = median(qvw_fp(:,4));

Qvw_fp = [q0,q1,q2,q3];
end