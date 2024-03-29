clear all;
close all;

%% IDMT Calc
i_k=0.14;
i_a=0.02;
vi_k=13.5;
vi_a=1;
ei_k=80;
ei_a=2;

current = 1.03:0.01:20;
currentSize=length(current);
i_t=zeros(1,currentSize);
vi_t=zeros(1,currentSize);
ei_t=zeros(1,currentSize);

for count=1:currentSize
   i_t(count)=round((i_k/(current(count)^i_a-1))*1000);
   vi_t(count)=round((vi_k/(current(count)^vi_a-1))*1000);
   ei_t(count)=round((ei_k/(current(count)^ei_a-1))*1000);
end
dlmwrite('it.csv',i_t);
csvwrite('vit.csv',vi_t);
csvwrite('eit.csv',ei_t);
%% Store in ms

intSize=20;
idmt=zeros(1,intSize);
vidmt=zeros(1,intSize);
eidmt=zeros(1,intSize);

%Storing the ingeter values in milliseconds
idmt(1,1)=round(i_t(1)*1000);
idmt(2,1)=1.03;
vidmt(1)=round(vi_t(1)*1000);
vidmt(2,1)=1;
eidmt(1)=round(ei_t(1)*1000);
eidmt(2,1)=1;
for count = 2:20
    position = find(current==count);
    idmt(1,count)=round(i_t(position)*1000);
    idmt(2,count)=current(position);
    vidmt(1,count)=round(vi_t(position)*1000);
    vidmt(2,count)=current(position);
    eidmt(1,count)=round(ei_t(position)*1000);
    eidmt(2,count)=current(position);
end

finalResult=zeros(1,190);
result =0;
val = 1.1:0.1:20
for count=1:190;
    xi = val(count)
    for i = 2:20
        term=idmt(1,i);
        for j=2:20
            if j~=i
                term = term*(xi-idmt(2,j))/(idmt(2,i)-idmt(2,j));
            end
        end
        result = result + term;
    end
    finalResult(count) = round(result);
    result=0;
end
    

figure(1)
subplot(121)
semilogy(current,i_t,'c',current,vi_t,'r',current,ei_t,'b');
xlabel('current (Irms)');
ylabel('time (s)');
grid;
subplot(122)
semilogy(val,finalResult,'c')
xlabel('current (Irms)');
ylabel('time (ms)');
grid;

%% test
xi = 1.04;
result = 0;
    for i = 1:20
        term=idmt(1,i);
        for j=1:20
            if j~=i
%                 temp =xi-idmt(2,j);
%                 temp2 =(idmt(2,i)-idmt(2,j));
%                 term=term*temp;
                term = term*(xi-idmt(2,j))/(idmt(2,i)-idmt(2,j));
            end
        end
        result = result + term;
    end
    
    round(result)
    
    
    
    
    
    
    
    
    
    