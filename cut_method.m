clc
clear all
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
image=imread('test.jpg');
image=image(:,:,1);
imshow(image)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1=[106,68,92,145,145,106]-22;
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
box on;
xlim([0,1000]);
ylim([0,500]);
hold on
patch(x1,y1,'k')
patch(x2,y2,'k')
patch(x3,y3,'k')
myConnect=[1,5;1,6;3,5;3,9;2,6;2,7;2,8;9,14;13,17;13,15;12,15;12,4;8,11;2,11];
pos=[0,0,1000,1000,x1,x2,x3;500,0,500,0,y1,y2,y3]';
for i=1:length(myConnect)
    line([pos(myConnect(i,1),1),pos(myConnect(i,2),1)],[pos(myConnect(i,1),2),pos(myConnect(i,2),2)])
end
regionTri=[1,5,6;1,6,2;2,6,7;2,7,8;2,8,11;12,13,15;15,13,17;1,5,3];
regionAve=zeros(13,2);
for i=1:length(regionTri)
    regionAve(i,1)=(pos(regionTri(i,1),1)+pos(regionTri(i,2),1)+pos(regionTri(i,3),1))/3;
    regionAve(i,2)=(pos(regionTri(i,1),2)+pos(regionTri(i,2),2)+pos(regionTri(i,3),2))/3;
end
regionRec=[2,4,11,12;4,15,16,12;8,9,11,14;9,13,17,3;5,10,9,3];
for i=length(regionTri)+1:length(regionTri)+length(regionRec)
    regionAve(i,1)=(pos(regionRec(i-8,1),1)+pos(regionRec(i-8,2),1)+pos(regionRec(i-8,3),1)+pos(regionRec(i-8,4),1))/4;
    regionAve(i,2)=(pos(regionRec(i-8,1),2)+pos(regionRec(i-8,2),2)+pos(regionRec(i-8,3),2)+pos(regionRec(i-8,4),2))/4;
end
conf=[(37-22)/339*1000,(204-84)/204*500;(347-22)/339*1000,(204-163)/204*500];
labels=cellstr(num2str([1:13]'));
plot(regionAve(:,1),regionAve(:,2),'r.','MarkerSize',19)
text(regionAve(:,1),regionAve(:,2),labels)
plot(conf(1,1),conf(1,2),'b.','MarkerSize',19)
text(conf(1,1),conf(1,2),cellstr('Initial'),'FontSize',8,'VerticalAlignment','bottom','HorizontalAlignment','center')
plot(conf(2,1),conf(2,2),'b.','MarkerSize',19)
text(conf(2,1),conf(2,2),cellstr('End'),'FontSize',8,'VerticalAlignment','bottom','HorizontalAlignment','center')
saveas(gcf,'setup','jpg')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dotConnect=zeros(13);
dotConnect(1,8)=1;
dotConnect(1,2)=1;
dotConnect(2,3)=1;
dotConnect(3,4)=1;
dotConnect(4,5)=1;
dotConnect(5,9)=1;
dotConnect(5,11)=1;
dotConnect(9,10)=1;
dotConnect(6,10)=1;
dotConnect(6,7)=1;
dotConnect(7,12)=1;
dotConnect(12,13)=1;
dotConnect(11,12)=1;
dotConnect(8,13)=1;
weight=zeros(1,14);
k=1;
for i=1:13
    for j=1:13
        if dotConnect(i,j)~=0
            weight(k)=sqrt((regionAve(i,1)-regionAve(j,1))^2+(regionAve(i,2)-regionAve(j,2))^2);
            k=k+1;
        end
    end
end
for i=1:13
    for j=1:13
        if dotConnect(i,j)>0
            dotConnect(j,i)=dotConnect(i,j);
        end
    end
end
G=graph(dotConnect);
G.Edges.Weight=weight';
figure;
P=plot(G,'EdgeLabel',round(G.Edges.Weight));
P.XData=regionAve(:,1);
P.YData=regionAve(:,2);
saveas(gcf,'dotConnect','jpg')
h=zeros(1,13);
for i=1:length(regionAve)
    h(i)=sqrt((regionAve(i,1)-regionAve(10,1))^2+(regionAve(i,2)-regionAve(10,2))^2);
end
gorder=zeros(13,13);
gDis=zeros(1,13);
for i=1:13
    [gtemp,gDis(i)]=shortestpath(G,2,i);
    gorder(i,1:length(gtemp))=gtemp;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
ini=2;
closeSet=[];
k=table2array(G.Edges);
f=zeros(1,13);
visited=zeros(1,13);
visited(2)=2;
Q=ones(13,1)*9999;
count=1;
nodes(1)=0;
gDis(ini)=0;
while ini~=10
    for j=1:13
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
route=zeros(1,13);
order=1;
ordertemp=10;
while ismember(2,route)==0
    for i=13:-1:1
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
figure(mainPlot);hold on
routeLength=0;
for i=2:length(route)
    line([regionAve(route(i-1),1),regionAve(route(i),1)],[regionAve(route(i-1),2),regionAve(route(i),2)],'LineWidth',3,'Color','r')
    routeLength=routeLength+sqrt((regionAve(route(i-1),1)-regionAve(route(i),1))^2+(regionAve(route(i-1),2)-regionAve(route(i),2))^2);
end
line([confx(1),regionAve(route(1),1)],[confy(1),regionAve(route(1),2)],'LineWidth',3,'Color','r')
line([confx(2),regionAve(route(end),1)],[confy(2),regionAve(route(end),2)],'LineWidth',3,'Color','r')
routeLength=routeLength+sqrt((confx(1)-regionAve(route(1),1))^2+(confy(1)-regionAve(route(1),2))^2)+sqrt((confx(2)-regionAve(route(end),1))^2+(confy(2)-regionAve(route(end),2))^2);
disp(route)
disp(routeLength)
saveas(gcf,'finished','jpg')