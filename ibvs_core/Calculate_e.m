function s = Calculate_e(m)
    s = zeros(numel(m),1);
    
    for i = 1:size(m,2)
        s(2*i-1,1) = m(1,i);
        s(  2*i,1) = m(2,i);
    end
    
end