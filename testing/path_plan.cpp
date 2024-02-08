#include <bits/stdc++.h> 
using namespace std; 

#define MAXN 100 
const int INF = 1e7; 

int dis[MAXN][MAXN]; 
int Next[MAXN][MAXN]; 


struct node {
	double x;
	double y;
	double z;
};

void initialise(int V,vector<vector<int>>& graph) 
{ 
	for (int i = 0; i < V; i++) { 
		for (int j = 0; j < V; j++) { 
			dis[i][j] = graph[i][j]; 
			if (graph[i][j] == INF) 
				Next[i][j] = -1; 
			else
				Next[i][j] = j; 
		} 
	} 
} 

vector<int> constructPath(int u, int v) 
{ 

	if (Next[u][v] == -1) 
		return {}; 
	vector<int> path = { u }; 
	while (u != v) { 
		u = Next[u][v]; 
		path.push_back(u); 
	} 
	return path; 
} 

void floydWarshall(int V) 
{ 
	for (int k = 0; k < V; k++) { 
		for (int i = 0; i < V; i++) { 
			for (int j = 0; j < V; j++) { 
				if (dis[i][k] == INF 
					|| dis[k][j] == INF) 

				if (dis[i][j] > dis[i][k] 
									+ dis[k][j]) { 
					dis[i][j] = dis[i][k] 
								+ dis[k][j]; 
					Next[i][j] = Next[i][k]; 
				} 
			} 
		} 
	} 
} 

vector<struct node*> nodes;


void create_graph(double x, double y,double z){

	struct node* temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 0; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 0; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 0; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 0; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 0; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 0; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 0; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 0; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 0; temp_node->z = 2*z;
	nodes.push_back(temp_node);




	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = y; temp_node->z = 2*z;
	nodes.push_back(temp_node);




	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 2*y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 2*y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 0; temp_node->y = 2*y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 2*y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 2*y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = x; temp_node->y = 2*y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = 0;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = z;
	nodes.push_back(temp_node);

	temp_node = new struct node;
	temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = 2*z;
	nodes.push_back(temp_node);

}


int find_distance(int point1, int point2){
	struct node node1;
	struct node node2;

	if(point1 ==0){
		node1.x = -1;
		node1.y = -1;
		node1.z = 0;
	}else{
		node1 = *nodes[point1-1];
	}	
	node2 = *nodes[point2-1];

	return (node1.x - node2.x)*(node1.x - node2.x) + (node1.y - node2.y)*(node1.y - node2.y) + (node1.z - node2.z)*(node1.z - node2.z);
}


vector<vector<int>> all;

void get_all_points(int* possible,int count,vector<int> &store){
	if(count == 9){
		vector<int> hello;
		for(int i =0;i<store.size();i++){
			if(store[i]!=0){
				hello.push_back(store[i]);
			}
		}
		all.push_back(hello);
		return;
	}
	store[count] = 0;
	get_all_points(possible,count+1,store);	
	store[count] = possible[count];
	get_all_points(possible,count+1,store);
}



int check_path(vector<int> path,int start_location){
	double sum=0;
	for(int i=0;i<path.size();i++){
		double val;
		if(i==0){
			val = find_distance(start_location,path[i]);
		}else{
			val = find_distance(path[i],path[i-1]);
		}sum = sum+val;
	}
	return sum;
}


int min_imum=INF;
vector<int> minimum_vector;
vector<vector<int>> minimums;


void permutations(vector<int> nums, int l, int h,int start_location) 
{ 	
    if (l == h){ 
		int tmp =check_path(nums,start_location);
		if(tmp<min_imum){
			minimum_vector = nums;
			min_imum = tmp;
		}		
        return; 
    } 

    for (int i = l; i <= h; i++) { 
        swap(nums[l], nums[i]);   
        permutations(nums, l + 1, h,start_location); 
        swap(nums[l], nums[i]); 
    } 
} 
  

void permute(vector<int>& nums,int start_location) 
{ 
    permutations(nums, 0,nums.size()-1,start_location); 
} 


vector<int> find_shortest_permutation(vector<int> points, int start_location){
		vector<int> min_vec;
		if(points.size()>2){
			permute(points,start_location);
			min_vec = minimum_vector;
		}
	return min_vec;	
}


void find_permutations(){
	int possible[9] ={1,2,3,4,5,6,7,8,9};
	vector<int> store(9);
	get_all_points(possible,0,store);
}


int find_next_plane_point(vector<int> plane0,vector<int> plane1){
	int min_dist=INF;
	int next;
	for(int i=0;i<plane1.size();i++){
		int tmp = find_distance(plane1[i],plane0[0]);
		if(min_dist>tmp){
			min_dist = find_distance(plane1[i],plane0[0]);
			next = plane1[i];
		}
	}
	return next;
}
	
vector<vector<double>> waypoint_matrix;

void get_waypoints(double inter_plant_dist){
	
		// int i=0;
	for(int i=0;i<nodes.size();i++){
		// cout<<nodes[i]->x<<" "<<nodes[i]->y<<" "<<nodes[i]->z<<endl;

		vector<double> waypoint1;
		waypoint1.push_back(nodes[i]->x-inter_plant_dist);waypoint1.push_back(nodes[i]->y);waypoint1.push_back(nodes[i]->z);
		
		vector<double> waypoint2;
		waypoint2.push_back(nodes[i]->x);waypoint2.push_back(nodes[i]->y);waypoint2.push_back(nodes[i]->z);

		vector<double> waypoint3;
		waypoint3.push_back(nodes[i]->x+inter_plant_dist);waypoint3.push_back(nodes[i]->y);waypoint3.push_back(nodes[i]->z);
		
		waypoint_matrix.push_back(waypoint1);
		waypoint_matrix.push_back(waypoint2);
		waypoint_matrix.push_back(waypoint3);
	}
}

void calculate_front(double distance){
	int length = waypoint_matrix.size();
	for(int i=0;i<length;i++){
		vector<double> waypoint = waypoint_matrix[i];
		waypoint_matrix.push_back(waypoint);
	}

	for(int i=0;i<length;i++){
		waypoint_matrix[i][0] = waypoint_matrix[i][0] - distance; 
	}													

	for(int i=length;i<2*length;i++){
		waypoint_matrix[i][0] = waypoint_matrix[i][0] + distance; 
	}
}

double calculate_waypoint_distance(vector<double> point1,vector<double> point2){
	double sum=0;
	for(int i=0;i<point1.size();i++){
		sum = sum + (point1[i]-point2[i])*(point1[i]-point2[i]);
	}
	return sum;
}


vector<vector<double>> translation(int shelf,vector<double> last_waypoint){

	vector<vector<double>> return_waypoints;
	vector<double> temp;
	if(shelf<0){
		for(int i=-1;i<2;i++){
			temp = waypoint_matrix[3*(-1*shelf-1)+1+i+waypoint_matrix.size()/2];
			return_waypoints.push_back(temp);
		}
	
	}else{
		for(int i=-1;i<2;i++){
			temp = waypoint_matrix[3*(shelf-1)+1+i];
			return_waypoints.push_back(temp);
		}
	}
	
	if(last_waypoint.empty()){
		return return_waypoints;
	}else{
		double distance_zero = calculate_waypoint_distance(last_waypoint,return_waypoints[0]);
		double distance_one = calculate_waypoint_distance(last_waypoint,return_waypoints[2]);
		if(distance_zero>distance_one){
			reverse(return_waypoints.begin(),return_waypoints.end());
			return return_waypoints;
		}else{
			return return_waypoints;
		}
	}
}


vector<vector<double>> convert_vector_to_waypoints(vector<int> shelves){
	vector<vector<double>> final_waypoints;
	vector<vector<double>> temp;

	for(int i=0;i<shelves.size();i++){
		if(i==0){
			vector<double> empty={};
			temp = translation(shelves[i],empty);

		}else{
			temp = translation(shelves[i],final_waypoints[final_waypoints.size()-1]);
		}


		for(int j=0;j<temp.size();j++){
			final_waypoints.push_back(temp[j]);
		}
	}

	return final_waypoints;
}

int main() 
{ 

	int V = 4; 
	vector<vector<int>> graph 
		= { { 0, 3, INF, 7 }, 
			{ 8, 0, 2, INF }, 
			{ 5, INF, 0, 1 }, 
			{ 2, INF, INF, 0 } }; 
	initialise(V, graph); 

	floydWarshall(V);
	vector<int> path;
	vector<int> points={1,5,2,6,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27};
	
	sort(points.begin(),points.end());

	// check_closest(points);
	vector<vector<int>> plane(3);

	for(int i = 0;i<points.size();i++){
		int plane_no = (points[i]-1)/9; 
		plane[plane_no].push_back(points[i]);
	}
	for(int i=0;i<plane.size();i++){
		cout << "Plane "<< i+1 << ": ";
		for(int j=0;j<plane[i].size();j++){
			cout<<plane[i][j]<<" ";
		}
		cout<<endl;
	}

	vector<int> final_path;

	create_graph(1,1,10);

	vector<int> plane0 = find_shortest_permutation(plane[0],0);


	for(int i =0;i<plane0.size();i++){
		final_path.push_back(plane0[i]);
	}

	for(int i=0;i<plane0.size();i++){
		final_path.push_back(-1*plane0[plane0.size()-1-i]);
	}

	int next = find_next_plane_point(plane0,plane[1]);

// // Reduntant Code Sorry
	min_imum = INF;

	vector<int> plane1 = find_shortest_permutation(plane[1],next);
	

	for(int i =0;i<plane1.size();i++){
		final_path.push_back(plane1[i]);
	}

	for(int i=0;i<plane1.size();i++){
		final_path.push_back(-1*plane1[plane1.size()-1-i]);
	}

	next = find_next_plane_point(plane1,plane[2]);

// // Reduntant Code Sorry

	min_imum = INF;

	vector<int> plane2 = find_shortest_permutation(plane[2],next);

	for(int i =0;i<plane2.size();i++){
		final_path.push_back(plane2[i]);
	}

	for(int i=0;i<plane2.size();i++){
		final_path.push_back(-1*plane2[plane2.size()-1-i]);
	}

	cout << endl;
	cout << "Final index of shelf in order: ";
	for(int i=0;i<final_path.size();i++){
		cout<<final_path[i]<<" ";
	}

	get_waypoints(7.0);
	calculate_front(6);
	vector<vector<double>> final_waypoints = convert_vector_to_waypoints(final_path);
	cout<<endl<<endl<< "Total no of points with plant:" <<final_path.size();

	for(int i = 0; i < final_path.size(); i++){
		if(final_path[i] > 0){
			for(int k = 0; k<3; k++){
				final_waypoints[(3*i)+k].push_back(0);
			}
		}
		else{
			for(int k = 0; k<3; k++){
				final_waypoints[(3*i)+k].push_back(180);
			}
		}
	}

	cout << endl << endl;

	for(int i =0; i<final_waypoints.size(); i++){
		cout << "Point Coordinate " << i+1 << ": ";
		for(int j=0; j<final_waypoints[i].size(); j++){
			cout << " " << final_waypoints[i][j] << " ";
		}
		cout << endl;
	}

	cout << endl;

	// cout << final_waypoints.size();
	
	return 0; 
}
