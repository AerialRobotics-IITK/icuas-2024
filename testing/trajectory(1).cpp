#include <bits/stdc++.h> 
using namespace std;

int main(){
    vector<int> points = {3,6,1,13,7,15,18,20};
    vector<int> Plane1;
    vector<int> Plane2;
    vector<int> Plane3;
    vector<vector<float>> Plane1_coordinates;
    vector<vector<float>> Plane2_coordinates;
    vector<vector<float>> Plane3_coordinates;

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
    cout << endl << endl;

    for(int i = 0; i<Plane1.size(); i++){
        for(int j = -1; j<=1; j++){
            float x = 2;
            float y;
            float z;
            if(Plane1[i]/3<=1){
                y = 6 + (j*1.5);
            }
            else if(Plane1[i]/3<=2){
                y = 13.5 + (j*1.5);
            }
            else{
                y = 21 + (j*1.5);
            }

            if(Plane1[i]%3==1){
                z = 1.1;
            }
            else if(Plane1[i]%3==2){
                z = 3.9;
            }
            else{
                z = 6.7;
            }

            Plane1_coordinates.push_back({x,y,z,0});
        }
        
    }

    for(int i = Plane1_coordinates.size()-1; i>=0; i--){
        Plane1_coordinates.push_back({Plane1_coordinates[i][0]+4,Plane1_coordinates[i][1],Plane1_coordinates[i][2],Plane1_coordinates[i][3]+180});
    }

    for(int i = 0; i<Plane2.size(); i++){
        for(int j = -1; j<=1; j++){
            float x = 8;
            float y;
            float z;
            if((Plane2[i]-9)/3<=1){
                y = 6 + (j*1.5);
            }
            else if((Plane2[i]-9)/3<=2){
                y = 13.5 + (j*1.5);
            }
            else{
                y = 21 + (j*1.5);
            }

            if(Plane2[i]%3==1){
                z = 1.1;
            }
            else if(Plane2[i]%3==2){
                z = 3.9;
            }
            else{
                z = 6.7;
            }

            Plane2_coordinates.push_back({x,y,z,0});
        }  
    }
    for(int i = Plane2_coordinates.size()-1; i>=0; i--){
        Plane2_coordinates.push_back({Plane2_coordinates[i][0]+4,Plane2_coordinates[i][1],Plane2_coordinates[i][2],Plane2_coordinates[i][3]+180});
    }

    for(int i = 0; i<Plane3.size(); i++){
        for(int j = -1; j<=1; j++){
            float x = 14;
            float y;
            float z;
            if((Plane3[i]-18)/3<=1){
                y = 6 + (j*1.5);
            }
            else if((Plane3[i]-18)/3<=2){
                y = 13.5 + (j*1.5);
            }
            else{
                y = 21 + (j*1.5);
            }

            if(Plane3[i]%3==1){
                z = 1.1;
            }
            else if(Plane3[i]%3==2){
                z = 3.9;
            }
            else{
                z = 6.7;
            }

            Plane3_coordinates.push_back({x,y,z,0});
        }        
    }

    for(int i = Plane3_coordinates.size()-1; i>=0; i--){
        Plane3_coordinates.push_back({Plane3_coordinates[i][0]+4,Plane3_coordinates[i][1],Plane3_coordinates[i][2],Plane3_coordinates[i][3]+180});
    }

    // cout << Plane1_coordinates.size() << endl;
    // cout << Plane1.size() << endl;

    for(int i = 0; i<Plane1_coordinates.size()/2; i++){
        cout << "Shelf " << Plane1[i/3] << " Plant " << (i%3)+1 <<" front: " << Plane1_coordinates[i][0] << " " << Plane1_coordinates[i][1] << " " << Plane1_coordinates[i][2]<<" "<<Plane1_coordinates[i][3] << endl;
    }
    cout << "\nBetween points\n\n";
    for(int i = Plane1_coordinates.size()/2; i<Plane1_coordinates.size(); i++){
        cout << "Shelf " << Plane1[i/3] << " Plant " << (i%3)+1 << " back: " << Plane1_coordinates[i][0] << " " << Plane1_coordinates[i][1] << " " << Plane1_coordinates[i][2]<<" "<<Plane1_coordinates[i][3] << endl;
    }
    cout << "Plane1 end\n\n";
    cout << "\nBetween points\n\n";
    for(int i = 0; i<Plane2_coordinates.size()/2; i++){
        cout << "Shelf " << Plane2[i/3] << " Plant " << (i%3)+1 << " front: " << Plane2_coordinates[i][0] << " " << Plane2_coordinates[i][1] << " " << Plane2_coordinates[i][2]<<" "<<Plane1_coordinates[i][3]  << endl;
    }
    cout << "\nBetween points\n\n";
    for(int i = Plane2_coordinates.size()/2; i<Plane2_coordinates.size(); i++){
        cout << "Shelf " << Plane2[i/3] << " Plant " << (i%3)+1 << " back: " << Plane2_coordinates[i][0]<< " " << Plane2_coordinates[i][1] << " " << Plane2_coordinates[i][2]<<" "<<Plane1_coordinates[i][3] << endl;
    }
    cout << "Plane2 end\n\n";
    cout << "\nBetween points\n\n";
    for(int i = 0; i<Plane3_coordinates.size()/2; i++){
        cout << "Shelf " << Plane3[i/3] << " Plant " << (i%3)+1 << " front: " << Plane3_coordinates[i][0] << " " << Plane3_coordinates[i][1] << " " << Plane3_coordinates[i][2]<<" "<<Plane1_coordinates[i][3]  << endl;
    }
    cout << "\nBetween points\n\n";
    for(int i = Plane3_coordinates.size()/2; i<Plane3_coordinates.size(); i++){
        cout << "Shelf " << Plane3[i/3] << " Plant " << (i%3)+1 << " back: " << Plane3_coordinates[i][0]<< " " << Plane3_coordinates[i][1] << " " << Plane3_coordinates[i][2]<<" "<<Plane1_coordinates[i][3] << endl;
    }
    cout << "Plane3 end\n\n";

    return 0;
}