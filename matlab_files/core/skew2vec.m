function vec=skew2vec(sw)
 
    vec(1,1) = sw(3,2);
    vec(1,2) = sw(1,3);
    vec(1,3) = sw(2,1);
    
end