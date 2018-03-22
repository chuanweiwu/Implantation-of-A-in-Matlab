clc
clear all
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
image=imread('test.jpg');
image=image(:,:,1);
% imshow(image)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1=[106,68,92,145,145,107]-22;
x1=x1/339*1000;
y1=204-[44,84,112,112,84,84];
y1=y1/204*500;
x2=[165,254,254,195]-22;
x2=x2/339*1000;
y2=204-[157,157,84,84];
y2=y2/204*500;
x3=[312,361,361]-22;
x3=x3/339*1000;
y3=204-[122,122,32];
y3=y3/204*500;
confx=[37,348]-22;
confx=confx/339*1000;
confy=204-[83,164];
confy=confy/204*500;
mainPlot=figure;
hold on;
patch(x1,y1,'k');
patch(x2,y2,'k')
patch(x3,y3,'k')
box on;
xlim([0,1000]);
ylim([0,500]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=1;
j=1;
k=1;
totalPoint=50;
w=0.4;
piB=round(w*totalPoint);
piU=totalPoint-piB;
xu=zeros(1,piB);
yu=zeros(1,piB);
xb=zeros(1,piU);
yb=zeros(1,piU);
while i<=totalPoint
    ini(1)=rand(1)*1000;ini(2)=rand(1)*500;
    if j<=piB
        if inpolygon(ini(1),ini(2),x1,y1)==1 || inpolygon(ini(1),ini(2),x2,y2)==1 || inpolygon(ini(1),ini(2),x3,y3)==1
            mu=[ini(1),ini(2)];
            sigma=[50000,0;0,50000];
            guess=mvnrnd(mu,sigma,1);
            if inpolygon(guess(1),guess(2),x1,y1)==1 || inpolygon(guess(1),guess(2),x2,y2)==1 || inpolygon(guess(1),guess(2),x3,y3)==1
                q=[(guess(1)+ini(1))/2,(guess(2)+ini(2))/2];
                if inpolygon(q(1),q(2),x1,y1)==0 && inpolygon(q(1),q(2),x2,y2)==0 && inpolygon(q(1),q(2),x3,y3)==0
                    xb(i)=q(1);yb(i)=q(2);
                    i=i+1;j=j+1;             
                end
            end
        elseif inpolygon(ini(1),ini(2),x1,y1)==0 && inpolygon(ini(1),ini(2),x2,y2)==0 && inpolygon(ini(1),ini(2),x3,y3)==0
            if k<=piU
            xu(i)=ini(1);yu(i)=ini(2);
            i=i+1;k=k+1;
            end
        end
    end
end    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=[confx(1),xu,xb(length(xu)+1:end),confx(2)];
y=[confy(1),yu,yb(length(xu)+1:end),confy(2)];
labels=cellstr(num2str([1:totalPoint+2]'));
plot(x,y,'r.','MarkerSize',19)
text(x,y,labels)
plot(xu,yu,'b.','MarkerSize',19)
plot(confx,confy,'b.','MarkerSize',29)
% saveas(gcf,'setup_2','jpg')
dotConnect=zeros(totalPoint);
for i=1:totalPoint+2
    for j=1:totalPoint+2
        if i~=j
            dotConnect(i,j)=isempty(polyxpoly([x(i),x(j)],[y(i),y(j)],x1,y1))&&isempty(polyxpoly([x(i),x(j)],[y(i),y(j)],x2,y2))&&isempty(polyxpoly([x(i),x(j)],[y(i),y(j)],x3,y3));
            if dotConnect(i,j)==1
                line([x(i),x(j)],[y(i),y(j)],'LineWidth',0.2)
            end
        end
    end
end
weight=zeros(1,sum(sum(dotConnect))/2);
k=1;
for i=1:totalPoint+2
    for j=1:totalPoint+2
        if dotConnect(i,j)~=0&&i<j
            weight(k)=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2);
            k=k+1;
        end
    end
end
G=graph(dotConnect);
G.Edges.Weight=weight';
h=zeros(1,totalPoint+2);
for i=1:totalPoint+2
    h(i)=sqrt((x(i)-x(end))^2+(y(i)-y(end))^2);
end
gorder=zeros(totalPoint+2,totalPoint+2);
for i=1:totalPoint+2
    [gtemp,~]=shortestpath(G,1,i);
    gorder(i,1:length(gtemp))=gtemp;
end
% saveas(gcf,'dotConnect_2','jpg')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
ini=1;
closeSet=[];
k=table2array(G.Edges);
f=zeros(1,totalPoint+2);
visited=zeros(1,totalPoint+2);
visited(1)=1;
Q=ones(totalPoint+2,1)*9999;
count=1;
nodes(1)=0;
gDis(ini)=0;
while ini~=totalPoint+2
    for j=1:totalPoint+2
        if dotConnect(ini,j)==1 && ismember(j,closeSet)==0
            [a1,~]=find(k==ini);
            [b1,~]=find(k==j);
            if visited(j)~=1
                gDis(j)=gDis(ini)+k(intersect(a1,b1),3);
                nodes(j)=ini;               
                visited(j)=1;                
                f(j)=gDis(j)+h(j);
                Q(j)=f(j);
            end
        end
    end
    count=count+1;
    closeSet(count)=ini;   
    [~,ini]=min(Q); 
    Q(ini)=9999;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;hold on
[xnodes,ynodes] = treelayout(nodes);
for i=1:length(xnodes)
    text(xnodes(i),ynodes(i),num2str(i))
end
treeplot(nodes)
route=zeros(1,totalPoint+2);
order=1;
ordertemp=totalPoint+2;
while ismember(1,route)==0
    for i=totalPoint+2:-1:1
        if ordertemp==i
            ordertemp=nodes(i);
            route(order)=i;
            order=order+1;
        end
    end
end
route=flipud(route(route~=0)')';
toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
routeLength=0;
figure(mainPlot);hold on
for i=2:length(route)
    line([x(route(i-1)),x(route(i))],[y(route(i-1)),y(route(i))],'LineWidth',5,'Color','g')
    routeLength=routeLength+sqrt((x(route(i-1))-x(route(i)))^2+(y(route(i-1))-y(route(i)))^2);
end
disp(route)
disp(routeLength)
% saveas(gcf,'finish_2','jpg')
close all