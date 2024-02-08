#include <bits/stdc++.h> 
using namespace std;

int main(){
    vector<int> points = {3,6,1,13,7,15,18,20};
    vector<int> Plane1;
    vector<int> Plane2;
    vector<int> Plane3;

    sort(points.begin(),points.end());

    for(int i =0; i<points.size();i++){
        if(points[i]<=9){
            Plane1.push_back(points[i]);
        }
        else if(points[i]<=18){
            Plane2.push_back(points[i]);
        }
        else{
            Plane3.push_back(points[i]);
        }
    }

    cout << "Plane 1 : ";
    for (int i = 0; i<Plane1.size(); i++){
        cout << Plane1[i]<<" ";
    }
    cout << endl;

    cout << "Plane 2 : " ;
    for (int i = 0; i<Plane2.size(); i++){
        cout << Plane2[i] << " ";
    }
    cout << endl;

    cout << "Plane 3 : ";
    for (int i = 0; i<Plane3.size(); i++){
        cout << Plane3[i]<< " ";
    }
    cout << endl;

    for(int i = 0; i<2;)

    return 0;
}