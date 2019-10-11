function [x_best,y_best, c_best] = ANMS(cm,Nbest)
lm = imregionalmax(cm);
[x,y] = find(lm);
N_Strong = size(x,1);
r = inf(1,N_Strong);
ED = 0;

for i = 1:N_Strong
    for j = 1:N_Strong
        if cm(x(j),y(j)) > cm(x(i),y(i))
           ED =  (x(j)-x(i))^2 + (y(j)-y(i))^2;
           if ED < r(i)
              r(i) = ED;
           end 
        end
    end
end    
[sort_r,index] = sort(r,'descend');
c_best = zeros(2,Nbest);

for i = 1:Nbest
    c_best(1,i) = x(index(i)); 
    c_best(2,i) = y(index(i));
end

x_best = c_best(1,:);
y_best = c_best(2,:);
end