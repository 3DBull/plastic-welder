%Arduino record
% nameidx = 0;
% while nameidx == 0
%     disp('Waiting for comms.. ..');
%     comms = seriallist();
%     nameidx = getnameidx(comms, 'COM6')
% end 
% disp('Found COM6')  
data = [];
exit = 0;
s = serial('COM7');
shape = [2,1];
while exit ==0
    
%     while s.status == 'closed'
%     fopen(s);
%     end
fopen(s);
byt = fscanf(s,'%f,%f',shape)
%d = str2num(byt);
    if length(byt)<1
        exit = 1;
    else 
        data = [data, byt];
    end
    fclose(s);
    
end
delete(s);
clear s;
t = 0.5:0.5:length(data)/2;

plot(t,data(1,:),t,data(2,:));

