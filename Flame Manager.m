clc; 
clear all; 
close all force;   
dim1=480;
dim2=640;
diff=zeros(dim1,dim2);
mode=1;
motion_thresh=20;
%joy = vrjoystick(1,'forcefeedback');
%ser=Bluetooth('HC-06',1);
%ser=serial('COM40','BaudRate',9600);
%fopen(ser);
img=uint8(zeros(dim1,dim2));
rold=int16(zeros(dim1,dim2));
gold=int16(zeros(dim1,dim2));
bold=int16(zeros(dim1,dim2));   
m=[2,4,5,4,2;4,9,12,9,4;5,12,15,12,5;4,9,12,9,4;2,4,5,4,2];
n=sum(m(:));  
start=floor(max(size(m))/2);
disp('Operating in manual mode');
tic;
while(1)      
    if(mode==1)                        %Automatic mode
        img=imread('C:/Users/Ashwin Vinoo/Desktop/abc.jpg','jpeg');
        r=img(1:dim1,1:dim2,1);    
        g=img(1:dim1,1:dim2,2);   
        b=img(1:dim1,1:dim2,3); 
        r=conv2(single(r),m)/n;
        g=conv2(single(g),m)/n;
        b=conv2(single(b),m)/n;
        r=int16(r(start+1:dim1+start,start+1:dim2+start));
        g=int16(g(start+1:dim1+start,start+1:dim2+start));
        b=int16(b(start+1:dim1+start,start+1:dim2+start));
        final=int16(zeros(dim1,dim2));
        for i=1:dim1
            k=0;
            k1=0;
            k2=0;
            for j=1:dim2
                r1=r(i,j);
                g1=g(i,j);
                b1=b(i,j);
               if(r1>200&&r1~=255&&r1>g1+15&&g1>b1+3)
                   k=k+1;
               elseif(k>=4&&r1==255&&r1>g1&&g1>b1+3)
                    k1=k1+1;
                    final(i,j)=1;
               elseif(k1>=2&&r1==255&&g1==255&&g1>b1)
                    k2=k1+1;
               elseif(k2>=2&&r1==255&&g1==255&&b1==255)
                   final(i,j)=1;
               else
                   k=0;
                   k1=0;
                   k2=0;
                end;
            end;       
        end;
        for i=1:dim1
            k=0;
            k1=0;
            k2=0;
            for j=dim2:-1:1
                r1=r(i,j);
                g1=g(i,j);
                b1=b(i,j);
                if(r1>200&&r1~=255&&r1>g1+15&&g1>b1+3)
                    k=k+1;                
                elseif(k>=4&&r1==255&&r1>g1&&g1>b1+3)                
                    k1=k1+1;
                elseif(k1>=2&&r1==255&&g1==255&&g1>b1)
                    k2=k1+1;
                elseif(k2>=2&&r1==255&&g1==255&&b1==255)
                    final(i,j)=1;
                else
                    k=0;
                    k1=0;
                    k2=0;
                end;
            end;       
        end;
        diff=(abs(rold-r)+abs(gold-g)+abs(bold-b)).*final;
        if(max(diff(:))>motion_thresh)
            for i=1:dim1
                if(max(diff(i,:))>motion_thresh)
                   topmost=i;
                   break;
                end;
            end;
            for i=dim1:-1:1
                if(max(diff(i,:))>motion_thresh)
                    bottommost=i;
                    break;
                end;
            end;
            for i=1:dim2
                if(max(diff(:,i))>motion_thresh)
                    leftmost=i;
                    break;
                end;
            end;
            for i=dim2:-1:1
                if(max(diff(:,i))>motion_thresh)
                    rightmost=i;
                    break;
                end;
            end;
            for i=leftmost:rightmost
                img(topmost,i,:)=[0,0,255];
                img(bottommost,i,:)=[0,0,255];
            end;
            for i=topmost:bottommost
                img(i,rightmost,:)=[0,0,255];
                img(i,leftmost,:)=[0,0,255];
            end; 
        rold=r;
        gold=g;
        bold=b;
        axes=(rightmost+leftmost)/2; 
        %fwrite(ser,[1,axes<320,abs(320-axes)/2.2],'uint8');
        else
        %    fwrite(ser,[2,0,0],'uint8');  
        end;
    elseif(mode==0)    %Manual mode
        if(povs==0)
            povs=1;
        elseif(povs==90)
            povs=2;
        elseif(povs==180)
            povs=3;
        elseif(povs==270)
            povs=4;
        else
            povs=0;
        end;
        fwrite(ser,[0,buttons(1),povs],'uint8');
    else              %Voice recogition mode
        img=imread('http://192.168.0.105:8080/shot.jpg','jpeg');
        fwrite(ser,[3,0,0],'uint8'); 
    end;   
    imshow(img,[0,255]);    
    getframe;
end;