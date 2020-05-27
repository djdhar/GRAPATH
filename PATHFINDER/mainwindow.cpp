#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QImage>
#include <iostream>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintDevice>
#include <QPoint>
#include<stdlib.h>

class MyPoint{
    public:
    QPair<int,int> YX;
    QColor color;
    MyPoint(){
    }

    MyPoint(QPair<int,int> &YXi, QColor colori){
        YX=YXi;
        color=colori;
    }
    bool operator>(MyPoint YXi){
        return (this->YX>YXi.YX);
    }
    bool operator<(MyPoint YXi){
        return (this->YX<YXi.YX);
    }
    bool operator<=(MyPoint YXi){
        return (this->YX<=YXi.YX);
    }
};
bool ComparePoints(MyPoint a, MyPoint b){
    return a.YX<b.YX;
}
int pixelsize = 50;
QVector<QPair<int,int> >PolygonPoints;
QVector<QPair<int,int> >Base;
QVector<MyPoint > MyPoints;
QVector<QPair<int,int> >Window;
QMap<QPair<int,int>,bool> Visited, Bars;
QMap<QPair<int,int>,QPair<int,int>> Parent;
QMap<QPair<int,int>,int> ShortestPath;
QVector<QPair<int,int>> UnVisitedPoints;


QColor edgeColor(255,255,255);
#define size_of_the_frame 900
#define height_of_the_frame 900
#define width_of_the_frame 1500
#define PI 3.14159
#define DELAYTIME 50

QPair<int,int> StartNode(-1,-1);
QPair<int,int> EndNode(-1,-1);
int grid_count = (height_of_the_frame/pixelsize)*(width_of_the_frame/pixelsize);
int visited=0;
int barcount=0;

QImage GlobalImg;
QString msg("You Win!!!");
void plotLineLow( QVector<QPair<int,int> > &v,int x0,int y0, int x1,int y1);
void plotLineHigh( QVector<QPair<int,int> > &v,int x0,int y0, int x1,int y1);
void Bresenham( QVector<QPair<int,int> > &v,int x0,int y0, int x1,int y1);

bool isValid(int x,int y){
    if(x>=0 && y>=0 && x<width_of_the_frame && y<height_of_the_frame)
        return true;
    return false;
}
using namespace std;
double rad(int deg){
    return (PI/180)*deg;
}

inline void delay(int millisecondsWait)
{
    QEventLoop loop;
    QTimer t;
    t.connect(&t, &QTimer::timeout, &loop, &QEventLoop::quit);
    t.start(millisecondsWait);
    loop.exec();
}
void MainWindow::ClearTraversalPath(){
    for(int i=pixelsize/2;i<width_of_the_frame;i=i+pixelsize){
        for(int j=pixelsize/2;j<height_of_the_frame;j=j+pixelsize){
            if(sourceImage.pixelColor(i,j)==QColor(255,255,255) || sourceImage.pixelColor(i,j)==QColor(246,170,85))
                point(sourceImage,i,j,4,0,0,0);
        }
    }
}
void InitiateShortestPath(QPair<int,int> p){
    for(int i=pixelsize/2;i<width_of_the_frame;i=i+pixelsize){
        for(int j=pixelsize/2;j<height_of_the_frame;j=j+pixelsize){

            UnVisitedPoints.push_back(qMakePair(i,j));
            ShortestPath[qMakePair(i,j)]=INT_MAX;
        }
    }
    ShortestPath[p]=0;
}

void plotLineLow( QVector<QPair<int,int> > &v,int x0,int y0, int x1,int y1){
    int dx=x1-x0;
    int dy=y1-y0;
    int yi=1;
    if(dy<0){
        yi=-1;
        dy=-dy;
    }
    int D=2*dy-dx;
    int y=y0;
    for(int x=x0;x<=x1;x=x+1){
        v.push_back(qMakePair(x,y));
        if(D>0){
            y=y+yi;
            D=D-2*dx;
        }
        D=D+2*dy;
    }

}
 void plotLineHigh( QVector<QPair<int,int> > &v,int x0,int y0, int x1,int y1){
     int dx=x1-x0;
     int dy=y1-y0;
     int xi=1;
     if(dx<0){
         xi=-1;
         dx=-dx;
     }
     int D=2*dx-dy;
     int x=x0;
     for(int y=y0;y<=y1;y=y+1){
         v.push_back(qMakePair(x,y));
         if(D>0){
             x=x+xi;
             D=D-2*dy;
         }
         D=D+2*dx;
     }
 }

 void Bresenham( QVector<QPair<int,int> > &v,int x0,int y0, int x1,int y1){
     if(abs(y1-y0)<abs(x1-x0)){
         if(x0>x1)
             plotLineLow(v,x1,y1,x0,y0);
         else {
             plotLineLow(v,x0,y0,x1,y1);
         }
     }
     else{
         if(y0>y1)
             plotLineHigh(v,x1,y1,x0,y0);
         else {
             plotLineHigh(v,x0,y0,x1,y1);
         }
     }
 }

 int RotX(int x, int y,int x1, int y1, int deg=45){
     return x1+(x-x1)*cos(rad(deg))-(y-y1)*sin(rad(deg));
 }
 int RotY(int y, int x,int x1, int y1, int deg=45){
     return y1+(x-x1)*sin(rad(deg))+(y-y1)*cos((rad(deg)));
 }

 void MainWindow::PrintVectors(){
     ClearScreen();
     int x=2;
     for(int i=0;i<MyPoints.size();i++){
         int y =MyPoints[i].YX.first;
         cout<<x<<" "<<y<<endl;
         auto r = MyPoints[i].color.red();
         auto g = MyPoints[i].color.green();
         auto b = MyPoints[i].color.blue();
         point(sourceImage,x,y,3,r,g,b);
         for(int j= size_of_the_frame-1;j>=y;j=j-pixelsize)
             point(sourceImage,x,j,3,r,g,b);
         x+=pixelsize;
     }
 }

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    connect(ui->frame,SIGNAL(Mouse_Pos()),this,SLOT(Mouse_Pressed()));
    connect(ui->frame,SIGNAL(sendMousePosition(QPoint&)),this,SLOT(showMousePosition(QPoint&)));

    ClearScreen();
}
void MainWindow::ClearScreen(void){
    QImage img=QImage(width_of_the_frame,height_of_the_frame,QImage::Format_RGB888);
        img.fill(QColor(Qt::black).rgb());

        QPainter painter( &img );
        QPen pen(Qt::red, 1);
        painter.setPen(pen);
        for(int i=0;i<height_of_the_frame;i=i+pixelsize)
            painter.drawLine(0,i,width_of_the_frame-1,i);
        for(int i=0;i<width_of_the_frame;i=i+pixelsize)
            painter.drawLine(i,0,i,height_of_the_frame-1);

        painter.end();
         ui->frame->setPixmap(QPixmap::fromImage(img));
         sourceImage=img;
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::point(QImage&img,int x,int y,int r, int red=255, int green=255, int blue=255)
{

    int left=x/pixelsize;
    int right=y/pixelsize;
    for(int i=left*pixelsize+1;i<(left+1)*pixelsize;i++){
        for(int j=right*pixelsize+1;j<(right+1)*pixelsize;j++){
            img.setPixel(i,j,qRgb(red,green,blue));
        }
    }
    ui->frame->setPixmap(QPixmap::fromImage(img));
    sourceImage=img;
}

void MainWindow::showMousePosition(QPoint &pos)
{
    //ui->mouse_movement->setText(" X : "+QString::number(pos.x())+", Y : "+QString::number(pos.y()));
}
void MainWindow::Mouse_Pressed()
{
    if(ui->start_node->isChecked()){
        if(StartNode.first==-1 || StartNode.second==-1){
        point(sourceImage,ui->frame->x,ui->frame->y,3,255,255,0);
        p1.setX((((int)ui->frame->x)/pixelsize)*pixelsize+pixelsize/2);
        p1.setY((((int)ui->frame->y)/pixelsize)*pixelsize+pixelsize/2);
        StartNode.first = p1.x();
        StartNode.second = p1.y();
        }
    }
    if(ui->end_node->isChecked()){
        if(EndNode.first==-1 || EndNode.second==-1){
        point(sourceImage,ui->frame->x,ui->frame->y,3,0,255,255);
        p1.setX((((int)ui->frame->x)/pixelsize)*pixelsize+pixelsize/2);
        p1.setY((((int)ui->frame->y)/pixelsize)*pixelsize+pixelsize/2);
        EndNode.first = p1.x();
        EndNode.second = p1.y();
        }
    }
    if(ui->bar_node->isChecked()){
    point(sourceImage,ui->frame->x,ui->frame->y,3,0,255,0);
    p1.setX((((int)ui->frame->x)/pixelsize)*pixelsize+pixelsize/2);
    p1.setY((((int)ui->frame->y)/pixelsize)*pixelsize+pixelsize/2);
    if(sourceImage.pixelColor(p1.x(),p1.y())!=QColor(0,255,0)) barcount++;
    Bars[qMakePair(p1.x(),p1.y())]=true;
    }
}

void MainWindow:: SpeacialClearance(void){
    ClearScreen();
    Visited.clear();
    point(sourceImage,size_of_the_frame-2,size_of_the_frame-2,2,255,255,0);
}



void MainWindow::on_pushButton_clicked()
{
    SpeacialClearance();
}

bool MainWindow::RatInAMazeHashMap(int x, int y){

    if(x<0 || y<0 || x>size_of_the_frame || y>size_of_the_frame)
        return false;
    else if(sourceImage.pixelColor(x,y)==QColor(0,255,0)){
        return false;
    }
    else if(sourceImage.pixelColor(x,y)==QColor(255,255,0))
        return true;

    else if(Visited[qMakePair(x,y)]==false && (sourceImage.pixelColor(x,y)==QColor(0,0,0) || sourceImage.pixelColor(x,y)==QColor(255,255,255))&& x>0 && y>0 && x<size_of_the_frame && y<size_of_the_frame){
        point(sourceImage,x,y,3);
        delay(DELAYTIME);
        bool right =  RatInAMazeHashMap(x+pixelsize,y);
        if(right) return right;
        bool down =   RatInAMazeHashMap(x,y+pixelsize);
        if(down) return down;

        Visited[qMakePair(x,y)]=true;
        point(sourceImage,x,y,3,0,0,0);
        delay(DELAYTIME);
        return false;

    }
    else return false;
}

bool MainWindow::RatInAMaze(int x, int y){

    if(x<0 || y<0 || x>size_of_the_frame || y>size_of_the_frame)
        return false;
    else if(sourceImage.pixelColor(x,y)==QColor(0,255,0)){
        return false;
    }
    else if(sourceImage.pixelColor(x,y)==QColor(255,255,0))
        return true;

    else if((sourceImage.pixelColor(x,y)==QColor(0,0,0) || sourceImage.pixelColor(x,y)==QColor(255,255,255))&& x>0 && y>0 && x<size_of_the_frame && y<size_of_the_frame){
        point(sourceImage,x,y,3);
        delay(DELAYTIME);
        bool right =  RatInAMaze(x+pixelsize,y);
        if(right) return right;
        bool down =   RatInAMaze(x,y+pixelsize);
        if(down) return down;


        point(sourceImage,x,y,3,0,0,0);
        delay(DELAYTIME);
        return false;

    }
    else return false;
}

void MainWindow::on_pushButton_2_clicked()
{
    ui->label_2->setText("");
    ui->pushButton->setEnabled(false);
    Visited.clear();
    point(sourceImage,size_of_the_frame-2,size_of_the_frame-2,2,255,255,0);
    int x = pixelsize/2;
    int y = pixelsize/2;
    bool res = RatInAMaze(x,y);
    cout<<res<<endl;
    if(res)
        ui->label_2->setText("REACHED");
    else
         ui->label_2->setText("FAILED TO REACH");
    ui->pushButton->setEnabled(true);
}

void MainWindow::on_pushButton_3_clicked()
{
    ui->label_2->setText("");
    ui->pushButton->setEnabled(false);
    Visited.clear();
    point(sourceImage,size_of_the_frame-2,size_of_the_frame-2,2,255,255,0);
    int x = pixelsize/2;
    int y = pixelsize/2;
    bool res = RatInAMazeHashMap(x,y);
    cout<<res<<endl;
    if(res)
        ui->label_2->setText("REACHED");
    else
         ui->label_2->setText("FAILED TO REACH");
    ui->pushButton->setEnabled(true);
}
void MainWindow::insertionSort()
{
    int i, j;
    MyPoint key;
    for (i = 1; i < MyPoints.size(); i++)
    {
        key = MyPoints[i];
        j = i - 1;
        while (j >= 0 && MyPoints[j] > key)
        {
            MyPoints[j + 1] = MyPoints[j];
            PrintVectors();
            delay(DELAYTIME);
            j = j - 1;
        }
        MyPoints[j + 1] = key;
        PrintVectors();
        delay(DELAYTIME);
    }
}


void MainWindow::on_pushButton_4_clicked()
{
    ClearScreen();
    Bars.clear();
    barcount=0;
    StartNode.first=-1;
    StartNode.second=-1;
    EndNode.first=-1;
    EndNode.second=-1;
}

void MainWindow::merge(int l, int m, int r)
{
    int i, j, k;
    int n1 = m - l + 1;
    int n2 =  r - m;

    /* create temp arrays */
    MyPoint L[n1], R[n2];

    /* Copy data to temp arrays L[] and R[] */
    for (i = 0; i < n1; i++)
        L[i] = MyPoints[l + i];
    for (j = 0; j < n2; j++)
        R[j] = MyPoints[m + 1+ j];

    /* Merge the temp arrays back into arr[l..r]*/
    i = 0; // Initial index of first subarray
    j = 0; // Initial index of second subarray
    k = l; // Initial index of merged subarray
    while (i < n1 && j < n2)
    {
        if (L[i] <= R[j])
        {
            MyPoints[k] = L[i];
            i++;
            PrintVectors();
            delay(DELAYTIME);
        }
        else
        {
            MyPoints[k] = R[j];
            j++;
            PrintVectors();
            delay(DELAYTIME);
        }
        k++;
    }

    /* Copy the remaining elements of L[], if there
       are any */
    while (i < n1)
    {
        MyPoints[k] = L[i];
        PrintVectors();
        delay(DELAYTIME);
        i++;
        k++;
    }

    /* Copy the remaining elements of R[], if there
       are any */
    while (j < n2)
    {
        MyPoints[k] = R[j];
        PrintVectors();
        delay(DELAYTIME);
        j++;
        k++;
    }
}

/* l is for left index and r is right index of the
   sub-array of arr to be sorted */
void MainWindow::mergeSort(int l, int r)
{
    if (l < r)
    {
        // Same as (l+r)/2, but avoids overflow for
        // large l and h
        int m = l+(r-l)/2;

        // Sort first and second halves
        mergeSort(l, m);
        mergeSort(m+1, r);

        merge(l, m, r);
    }
}

void MainWindow::on_pushButton_5_clicked()
{
    point(sourceImage,StartNode.first,StartNode.second,0,0,0,0);
    StartNode = QPair<int,int> (-1,-1);
    point(sourceImage,EndNode.first,EndNode.second,0,0,0,0);
    EndNode = QPair<int,int> (-1,-1);
}

int MainWindow::partition (int low, int high)
{
    MyPoint pivot = MyPoints[high]; // pivot
    int i = (low - 1); // Index of smaller element

    for (int j = low; j <= high - 1; j++)
    {
        // If current element is smaller than the pivot
        if (MyPoints[j] < pivot)
        {
            i++; // increment index of smaller element
            swap(MyPoints[i], MyPoints[j]);
            PrintVectors();
            delay(DELAYTIME);

        }
    }
    swap(MyPoints[i + 1], MyPoints[high]);
    PrintVectors();
    delay(DELAYTIME);
    return (i + 1);
}

void MainWindow::quickSort(int low, int high)
{
    if (low < high)
    {
        int pi = partition(low, high);
        quickSort(low, pi - 1);
        quickSort(pi + 1, high);
    }
}

void MainWindow::on_pushButton_7_clicked()
{
    ClearTraversalPath();
}

void MainWindow::heapify(int n, int i)
{
    int largest = i;
    int l = 2*i + 1;
    int r = 2*i + 2;
    if (l < n && MyPoints[l] > MyPoints[largest])
        largest = l;
    if (r < n && MyPoints[r] > MyPoints[largest])
        largest = r;
    if (largest != i)
    {
        swap(MyPoints[i], MyPoints[largest]);
        PrintVectors();
        delay(DELAYTIME);
        heapify(n, largest);
    }
}

void MainWindow::heapSort(int n)
{
    for (int i = n / 2 - 1; i >= 0; i--)
        heapify(n, i);
    for (int i=n-1; i>0; i--)
    {
        swap(MyPoints[0], MyPoints[i]);
        PrintVectors();
        delay(DELAYTIME);
        heapify(i, 0);
    }
}

void MainWindow::on_pushButton_6_clicked()
{
    visited=0;
    Visited.clear();
    if(StartNode.first==-1 || StartNode.second==-1 || EndNode.first==-1 || StartNode.second==-1  ){
        cout<<"Choose First"<<endl;
        return;
    }
    if(ui->dfs->isChecked()){
        int found=0;
        DFS(StartNode.first,StartNode.second,found);
        cout<<"dfs"<<endl;
    }
    if(ui->bfs->isChecked()){
        int found=0;
        BFS(StartNode.first,StartNode.second,found);
        cout<<"bfs"<<endl;
    }
    if(ui->djktr->isChecked()){
        //InitiateShortestPath(StartNode);
        int found=0;


        RunDJKTR(StartNode.first,StartNode.second,found);
        cout<<"djktr"<<endl;
    }
    if(ui->bellmanford->isChecked()){
        BellManFord();
    }
    if(ui->floyd->isChecked()){
        FloydWarshall();
    }
}
bool MainWindow::FloydWarshall(){
    QMap<int,QPair<int,int>> map;
    QMap<QPair<int,int>,int> revmap;
    int p=0;
    for(int i=pixelsize/2;i<width_of_the_frame;i=i+pixelsize){
        for(int j=pixelsize/2;j<height_of_the_frame;j=j+pixelsize){
            map[p]=qMakePair(i,j);
            revmap[qMakePair(i,j)]=p;
            p++;
        }
    }

    int dist[map.size()][map.size()];
    int next[map.size()][map.size()];

    for(int i=0;i<map.size();i++){
        for(int j=0;j<map.size();j++){
            dist[i][j]=INT_MAX;
            next[i][j]=INT_MAX;
        }
    }
    for(int i=0;i<map.size();i++){
        QPair<int,int> vertex =map[i];
        int x=vertex.first;
        int y=vertex.second;
        if(isValid(x+pixelsize,y) ){
            if(sourceImage.pixelColor(x+pixelsize,y)!= QColor(0,255,0)){
            dist[i][revmap[qMakePair(x+pixelsize,y)]]=1;
            next[i][revmap[qMakePair(x+pixelsize,y)]]=revmap[qMakePair(x+pixelsize,y)];
        }
        }
        if(isValid(x-pixelsize,y)){
            if(sourceImage.pixelColor(x-pixelsize,y)!= QColor(0,255,0)){
            dist[i][revmap[qMakePair(x-pixelsize,y)]]=1;
            next[i][revmap[qMakePair(x-pixelsize,y)]]=revmap[qMakePair(x-pixelsize,y)];
        }
        }
        if(isValid(x,y+pixelsize)){
            if(sourceImage.pixelColor(x,y+pixelsize)!= QColor(0,255,0)){
            dist[i][revmap[qMakePair(x,y+pixelsize)]]=1;
            next[i][revmap[qMakePair(x,y+pixelsize)]]=revmap[qMakePair(x,y+pixelsize)];
        }
        }
        if(isValid(x,y-pixelsize)){
            if(sourceImage.pixelColor(x,y-pixelsize)!= QColor(0,255,0)){
            dist[i][revmap[qMakePair(x,y-pixelsize)]]=1;
            next[i][revmap[qMakePair(x,y-pixelsize)]]=revmap[qMakePair(x,y-pixelsize)];
        }
        }
        dist[i][i]=0;
        next[i][i]=i;
    }

    int gg=0;
    int N = (height_of_the_frame/pixelsize)*(width_of_the_frame/pixelsize);;
        for(int k=0;k<N;k++){
            int x=map[k].first, y=map[k].second;
            for(int i=0;i<N;i++){

                for(int j=0;j<N;j++){
                    if((long)dist[i][j]>(long)dist[i][k]+(long)dist[k][j]){
                        dist[i][j]=dist[i][k]+dist[k][j];
                        //point(sourceImage,x,y,2); delay(DELAYTIME);
                        cout<<gg<<endl;
                        gg++;
                        next[i][j]=next[i][k];
                    }
                }
            }
            if(sourceImage.pixelColor(x,y)!=QColor(0,255,0) && map[k]!=StartNode && map[k]!=EndNode)
                point(sourceImage,x,y,2); delay(DELAYTIME);
        }

        int u = revmap[StartNode];
        int v = revmap[EndNode];
        if(next[u][v]==INT_MAX)
            return false;
        if(dist[u][v]==INT_MAX)
            return false;
        int g=0;
        while(u!=v){
            g++;
            u=next[u][v];
            int x= map[u].first;
            int y = map[u].second;
            if(map[u]!=EndNode){
               point(sourceImage,x,y,2,246,170,85); delay(DELAYTIME);
            }
            if(g==900)
                break;
        }
    return true;
}
bool MainWindow::BellManFord(){

    QVector <QPair<QPair<int,int>,int>> M;
    QMap<int,QPair<int,int>> map;
    QMap<QPair<int,int>,int> revmap;
    int p=0;
    for(int i=pixelsize/2;i<width_of_the_frame;i=i+pixelsize){

        for(int j=pixelsize/2;j<height_of_the_frame;j=j+pixelsize){
            if(qMakePair(i,j)==EndNode){
                M.push_back(qMakePair(qMakePair(i,j),0));
            }
            else {
                M.push_back(qMakePair(qMakePair(-1,-1),INT_MAX));
            }
            map[p]=qMakePair(i,j);
            revmap[qMakePair(i,j)]=p;

            p++;
        }
    }
    cout<<"foo"<<endl;
    int total_itr =(height_of_the_frame/pixelsize)*(width_of_the_frame/pixelsize);
  cout<<"total itr ="<<total_itr<<" "<<map.size()<<endl;
   while(total_itr--){
       for(int i=0;i<map.size();i++){
           int x= map[i].first;
           int y = map[i].second;
           if(sourceImage.pixelColor(x,y)==QColor(0,255,0)){
               continue;
           }
           QPair<QPair<int,int>,int> det = qMakePair(qMakePair(-1,-1),INT_MAX);
           if(isValid(x-pixelsize,y) ){
               if(sourceImage.pixelColor(x-pixelsize,y)!=QColor(0,255,0)){
               if( M[revmap[qMakePair(x-pixelsize,y)]].first != qMakePair(-1,-1)){
                   if((long)M[revmap[qMakePair(x-pixelsize,y)]].second+(long)1<(long)det.second){
                       det.second=M[revmap[qMakePair(x-pixelsize,y)]].second+1;
                       det.first=qMakePair(x-pixelsize,y);
                   }
               }
                }
           }

           if(isValid(x+pixelsize,y) ){
               if(sourceImage.pixelColor(x+pixelsize,y)!=QColor(0,255,0)){
               if( M[revmap[qMakePair(x+pixelsize,y)]].first != qMakePair(-1,-1)){
                   if((long)M[revmap[qMakePair(x+pixelsize,y)]].second+(long)1<(long)det.second){
                       det.second=M[revmap[qMakePair(x+pixelsize,y)]].second+1;
                       det.first=qMakePair(x+pixelsize,y);
                   }
               }
               }
           }

           if(isValid(x,y+pixelsize) ){
               if(sourceImage.pixelColor(x,y+pixelsize)!=QColor(0,255,0)){
               if( M[revmap[qMakePair(x,y+pixelsize)]].first != qMakePair(-1,-1)){
                   if((long)M[revmap[qMakePair(x,y+pixelsize)]].second+(long)1<(long)det.second){
                       det.second=M[revmap[qMakePair(x,y+pixelsize)]].second+1;
                       det.first=qMakePair(x,y+pixelsize);
                   }
               }
               }
           }

           if(isValid(x,y-pixelsize) ){
               if(sourceImage.pixelColor(x,y-pixelsize)!=QColor(0,255,0)){
               if( M[revmap[qMakePair(x,y-pixelsize)]].first != qMakePair(-1,-1)){
                   if((long)M[revmap[qMakePair(x,y-pixelsize)]].second+(long)1<(long)det.second){
                       det.second=M[revmap[qMakePair(x,y-pixelsize)]].second+1;
                       det.first=qMakePair(x,y-pixelsize);
                   }
               }
               }
           }

           if(det.first!=qMakePair(-1,-1) && det.second<M[i].second){
               M[i]=det;
               if(qMakePair(x,y)!=StartNode && qMakePair(x,y)!=EndNode)
                point(sourceImage,x,y,2); delay(DELAYTIME);
           }
       }
   }
    bool found=false;

       int x=revmap[StartNode];
        int g=0;
       QPair<QPair<int,int>,int> det = M[x];
       while(det.first!=qMakePair(-1,-1)){
           g++;
           //cout<<"X = "<<det.first.first<<" Y = "<<det.first.second<<endl;
           if(det.first!=StartNode && det.first!=EndNode)
                point(sourceImage,det.first.first,det.first.second,2,246,170,85);
           delay(DELAYTIME);
           if(det.first==EndNode){
               found=true;
               break;
           }
           /*
           if(g==900)
               break;
           */
           det=M[revmap[det.first]];
       }


   return found;
}
bool MainWindow::RunDJKTR(int x,int y, int &found){
        Parent.clear();

        found = Dijkstra(StartNode.first,StartNode.second);
        cout<<found<<endl;
        if(found){
            QPair<int,int> finalNode =EndNode;
            while(1){
                int a = Parent[finalNode].first;
                int b = Parent[finalNode].second;
                cout<<a<<" "<<b<<endl;
                if(a==StartNode.first && b==StartNode.second){break;}
                else {point(sourceImage,a,b,4,246,170,85); delay(DELAYTIME);}
                finalNode=Parent[finalNode];
            }
        }
        return  found;
}

bool MainWindow::DJKTR(int x,int y, int &found){
    /*
    if(found)
        return true;
        */
     ui->show->setText("( "+QString::number(x)+" , "+QString::number(y)+" ) ");
    QPair<int,int> frnt(x,y);
    QVector<QPair<int,int>> adjacents;
    Visited[frnt]=true;
    QColor colr = sourceImage.pixelColor(x,y);
    point(sourceImage,x,y,4,255,0,0); delay(DELAYTIME/2); point(sourceImage,x,y,4,colr.red(),colr.green(),colr.blue()); delay(DELAYTIME/2);
    if(sourceImage.pixelColor(x,y) == QColor(0,255,0)){
        return false;
    }

    if(sourceImage.pixelColor(x,y) == QColor(0,255,255)){
        point(sourceImage,x,y,0,255,254); delay(DELAYTIME);
        found=1;
        //return true;
    }

    else if(x==StartNode.first && y==StartNode.second){}
    else {/*if(sourceImage.pixelColor(x,y)!=QColor(255,255,255)){}*/ visited++;    point(sourceImage,x,y,4); delay(DELAYTIME); }

    if((visited+barcount+2)==grid_count){
        found=1;
        cout<<visited<<endl;
        cout<<Bars.size()<<endl;
        cout<<grid_count<<endl;
        throw (404);
        return true;
    }

    if(isValid(x+pixelsize,y) /* && sourceImage.pixelColor(x_+pixelsize,y_)!=QColor(0,255,0)  && !Visited[qMakePair(x_+pixelsize,y_)]*/){
        QColor color= sourceImage.pixelColor(x+pixelsize,y);
        if(color!= QColor(255,255,255)){
            if(color != QColor(0,255,0) && color != QColor(0,255,254)){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x+pixelsize,y)]);
            if(sp!=ShortestPath[qMakePair(x+pixelsize,y)])
                Parent[qMakePair(x+pixelsize,y)]=qMakePair(x,y);
            ShortestPath[qMakePair(x+pixelsize,y)]=sp;
            adjacents.push_back(qMakePair(x+pixelsize,y));
        }
        }
    }
    if(isValid(x,y+pixelsize) /* && sourceImage.pixelColor(x_,y_+pixelsize)!=QColor(0,255,0)  && !Visited[qMakePair(x_,y_+pixelsize)]*/){
        QColor color= sourceImage.pixelColor(x,y+pixelsize);
        if(color != QColor(255,255,255)){
             if(color != QColor(0,255,0) && color != QColor(0,255,254)){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x,y+pixelsize)]);
            if(sp!=ShortestPath[qMakePair(x,y+pixelsize)])
                Parent[qMakePair(x,y+pixelsize)]=qMakePair(x,y);

            ShortestPath[qMakePair(x,y+pixelsize)]=sp;
            adjacents.push_back(qMakePair(x,y+pixelsize));
        }

        }
    }
    if(isValid(x-pixelsize,y) /* && sourceImage.pixelColor(x_-pixelsize,y_)!=QColor(0,255,0)  && !Visited[qMakePair(x_-pixelsize,y_)]*/){
        QColor color= sourceImage.pixelColor(x-pixelsize,y);
        if(color != QColor(255,255,255) && color != QColor(0,255,254)){
             if(color != QColor(0,255,0)){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x-pixelsize,y)]);
            if(sp!=ShortestPath[qMakePair(x-pixelsize,y)])
                Parent[qMakePair(x-pixelsize,y)]=qMakePair(x,y);
            ShortestPath[qMakePair(x-pixelsize,y)]=sp;

            adjacents.push_back(qMakePair(x-pixelsize,y));
        }

        }
    }
    if(isValid(x,y-pixelsize) /* && sourceImage.pixelColor(x_,y_-pixelsize)!=QColor(0,255,0)   && !Visited[qMakePair(x_,y_-pixelsize)]*/){
        QColor color= sourceImage.pixelColor(x,y-pixelsize);
        if(color != QColor(255,255,255)){
             if(color != QColor(0,255,0) && color != QColor(0,255,254)){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x,y-pixelsize)]);
            if(sp!=ShortestPath[qMakePair(x,y-pixelsize)])
                Parent[qMakePair(x,y-pixelsize)]=qMakePair(x,y);

            ShortestPath[qMakePair(x,y-pixelsize)]=sp;
            adjacents.push_back(qMakePair(x,y-pixelsize));
        }

        }
    }

    int sp =INT_MAX;
    cout<<adjacents.size()<<endl;
    if(!adjacents.size())
        return false;

    QVector<QPair<int,QPair<int,int>>> SortedAdjacents;
    for(int i=0;i<adjacents.size();i++){
        SortedAdjacents.push_back(qMakePair(ShortestPath[adjacents[i]],adjacents[i]));
    }
    sort(SortedAdjacents.begin(),SortedAdjacents.end());
    bool id=false;

    for(int i=0;i<SortedAdjacents.size();i++){
        QPair<int,int> pair = SortedAdjacents[i].second;
        id =id | DJKTR(pair.first,pair.second,found);
    }
    return id;



}
bool MainWindow::BFS(int x,int y, int &found){
    Parent.clear();
     ui->show->setText("( "+QString::number(x)+" , "+QString::number(y)+" ) ");
    if(!isValid(x,y))
        return false;
    if(sourceImage.pixelColor(x,y) == QColor(0,255,0)){
        return false;
    }
    if(sourceImage.pixelColor(x,y) == QColor(0,255,255)){
        found=1;
        return true;
    }
    Visited[qMakePair(x,y)]=true;
    if(x==StartNode.first && y==StartNode.second){}
    else {point(sourceImage,x,y,4); delay(DELAYTIME);}


    QVector<QPair<int,int>> adjacents;
    if(isValid(x+pixelsize,y) /* && sourceImage.pixelColor(x+pixelsize,y)!=QColor(0,255,0)*/ && !Visited[qMakePair(x+pixelsize,y)]){
        Parent[qMakePair(x+pixelsize,y)]=qMakePair(x,y);
        adjacents.push_back(qMakePair(x+pixelsize,y));
    }
    if(isValid(x,y+pixelsize)  /* && sourceImage.pixelColor(x,y+pixelsize)!=QColor(0,255,0)*/ && !Visited[qMakePair(x,y+pixelsize)]){
        Parent[qMakePair(x,y+pixelsize)]=qMakePair(x,y);
        adjacents.push_back(qMakePair(x,y+pixelsize));
    }
    if(isValid(x-pixelsize,y)/*  && sourceImage.pixelColor(x-pixelsize,y)!=QColor(0,255,0)*/ && !Visited[qMakePair(x-pixelsize,y)]){
        Parent[qMakePair(x-pixelsize,y)]=qMakePair(x,y);
        adjacents.push_back(qMakePair(x-pixelsize,y));
    }
    if(isValid(x,y-pixelsize)/* && sourceImage.pixelColor(x,y-pixelsize)!=QColor(0,255,0)*/ && !Visited[qMakePair(x,y-pixelsize)]){
        Parent[qMakePair(x,y-pixelsize)]=qMakePair(x,y);
        adjacents.push_back(qMakePair(x,y-pixelsize));
    }
    adjacents.pop_front();
    int i=0;
    while(adjacents.size() && !found){

        QPair<int,int> frnt = adjacents[i];

        int x_ = frnt.first;
        int y_ = frnt.second;
         ui->show->setText("( "+QString::number(x_)+" , "+QString::number(y_)+" ) ");
        if(x_==EndNode.first && y_==EndNode.second){
            found=1;
            break;
        }
        if(Bars[qMakePair(x_,y_)]){
            //i++;
            adjacents.pop_front();
            continue;
        }
        if(Visited[qMakePair(x_,y_)]){
            //i++;
            adjacents.pop_front();
            continue;
        }
        Visited[frnt]=true;
        if(x_==StartNode.first && y_==StartNode.second){}
        else {point(sourceImage,x_,y_,4); delay(DELAYTIME);}

        if(isValid(x_+pixelsize,y_) /* && sourceImage.pixelColor(x_+pixelsize,y_)!=QColor(0,255,0)  && !Visited[qMakePair(x_+pixelsize,y_)]*/){
            if(!Visited[qMakePair(x_+pixelsize,y_)]){
            Parent[qMakePair(x_+pixelsize,y_)]=frnt;
            adjacents.push_back(qMakePair(x_+pixelsize,y_));
            }
        }
        if(isValid(x_,y_+pixelsize) /* && sourceImage.pixelColor(x_,y_+pixelsize)!=QColor(0,255,0)  && !Visited[qMakePair(x_,y_+pixelsize)]*/){
            if(!Visited[qMakePair(x_,y_+pixelsize)]){
            Parent[qMakePair(x_,y_+pixelsize)]=frnt;
            adjacents.push_back(qMakePair(x_,y_+pixelsize));
            }
        }
        if(isValid(x_-pixelsize,y_) /* && sourceImage.pixelColor(x_-pixelsize,y_)!=QColor(0,255,0)  && !Visited[qMakePair(x_-pixelsize,y_)]*/){
            if(!Visited[qMakePair(x_-pixelsize,y_)]){
            Parent[qMakePair(x_-pixelsize,y_)]=frnt;
            adjacents.push_back(qMakePair(x_-pixelsize,y_));
            }
        }
        if(isValid(x_,y_-pixelsize) /* && sourceImage.pixelColor(x_,y_-pixelsize)!=QColor(0,255,0)   && !Visited[qMakePair(x_,y_-pixelsize)]*/){
            if(!Visited[qMakePair(x_,y_-pixelsize)]){
            Parent[qMakePair(x_,y_-pixelsize)]=frnt;
            adjacents.push_back(qMakePair(x_,y_-pixelsize));
            }
        }
        adjacents.pop_front();
        //i++;
    }
    if(found){
        QPair<int,int> finalNode =EndNode;
        while(1){
            int a = Parent[finalNode].first;
            int b = Parent[finalNode].second;
            cout<<a<<" "<<b<<endl;
            if(a==StartNode.first && b==StartNode.second){break;}
            else {point(sourceImage,a,b,4,246,170,85); delay(DELAYTIME);}
            finalNode=Parent[finalNode];
        }
    }
    return found;

}

bool MainWindow::DFS(int x,int y, int &found){
    ui->show->setText("( "+QString::number(x)+" , "+QString::number(y)+" ) ");
    if(!isValid(x,y))
        return false;
    if(sourceImage.pixelColor(x,y) == QColor(0,255,0)){
        return false;
    }
    if(sourceImage.pixelColor(x,y) == QColor(0,255,255)){
        found=1;
        return true;
    }
    Visited[qMakePair(x,y)]=true;
    if(x==StartNode.first && y==StartNode.second){}
    else point(sourceImage,x,y,4);
    delay(DELAYTIME);

    QVector<QPair<int,int>> adjacents;
    if(isValid(x+pixelsize,y)){
        adjacents.push_back(qMakePair(x+pixelsize,y));
    }
    if(isValid(x,y+pixelsize)){
        adjacents.push_back(qMakePair(x,y+pixelsize));
    }
    if(isValid(x-pixelsize,y)){
        adjacents.push_back(qMakePair(x-pixelsize,y));
    }
    if(isValid(x,y-pixelsize)){
        adjacents.push_back(qMakePair(x,y-pixelsize));
    }
    bool id=false;
    for(int i=0;i<adjacents.size();i++){
        if(!Visited[adjacents[i]] && !found){
            id =id | (DFS(adjacents[i].first,adjacents[i].second,found));
        }
    }
    if(id){
        if(x==StartNode.first && y==StartNode.second){}
        else point(sourceImage,x,y,4,246,170,85);
        delay(DELAYTIME);
    }

    return id;

}
QPair<int,int> GetMinimumWeight(int &index){
    int minW = INT_MAX;
    QPair<int,int> p;
    for(int i=0;i<UnVisitedPoints.size();i++){
        if(ShortestPath[UnVisitedPoints[i]]<minW){
            minW= ShortestPath[UnVisitedPoints[i]];
            index=i;
            p=UnVisitedPoints[i];
        }
    }
    return p;
}

bool MainWindow::Dijkstra(int x,int y){
    InitiateShortestPath(qMakePair(x,y));

    int index=0;

    while(UnVisitedPoints.size()){
        QPair<int,int> frnt = GetMinimumWeight(index);
        int x= frnt.first;
        int y = frnt.second;
        Visited[frnt]=true;

        if(qMakePair(x,y) == EndNode){
            return true;
        }

        else if(x==StartNode.first && y==StartNode.second){}
        else {/*if(sourceImage.pixelColor(x,y)!=QColor(255,255,255)){}*/ visited++;    point(sourceImage,x,y,4); delay(DELAYTIME); }


    if(isValid(x+pixelsize,y)){
        if(sourceImage.pixelColor(x+pixelsize,y)!=QColor(0,255,0)){
            if(!Visited[qMakePair(x+pixelsize,y)]){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x+pixelsize,y)]);
            if(sp!=ShortestPath[qMakePair(x+pixelsize,y)])
                Parent[qMakePair(x+pixelsize,y)]=qMakePair(x,y);
            ShortestPath[qMakePair(x+pixelsize,y)]=sp;
            }
        }
    }
    if(isValid(x,y+pixelsize)){
        if(sourceImage.pixelColor(x,y+pixelsize)!=QColor(0,255,0)){
             if(!Visited[qMakePair(x,y+pixelsize)]){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x,y+pixelsize)]);
            if(sp!=ShortestPath[qMakePair(x,y+pixelsize)])
                Parent[qMakePair(x,y+pixelsize)]=qMakePair(x,y);
            ShortestPath[qMakePair(x,y+pixelsize)]=sp;
             }
        }
    }
    if(isValid(x-pixelsize,y)){
        if(sourceImage.pixelColor(x-pixelsize,y)!=QColor(0,255,0)){
             if(!Visited[qMakePair(x-pixelsize,y)]){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x-pixelsize,y)]);
            if(sp!=ShortestPath[qMakePair(x-pixelsize,y)])
                Parent[qMakePair(x-pixelsize,y)]=qMakePair(x,y);
            ShortestPath[qMakePair(x-pixelsize,y)]=sp;
             }
        }
    }
    if(isValid(x,y-pixelsize)){
        if(sourceImage.pixelColor(x,y-pixelsize)!=QColor(0,255,0)){
             if(!Visited[qMakePair(x,y-pixelsize)]){
            int sp =min(ShortestPath[qMakePair(x,y)]+1,ShortestPath[qMakePair(x,y-pixelsize)]);
            if(sp!=ShortestPath[qMakePair(x,y-pixelsize)])
                Parent[qMakePair(x,y-pixelsize)]=qMakePair(x,y);
            ShortestPath[qMakePair(x,y-pixelsize)]=sp;
             }
        }
    }

    UnVisitedPoints.erase(UnVisitedPoints.begin()+index);
    }

    return false;
}
