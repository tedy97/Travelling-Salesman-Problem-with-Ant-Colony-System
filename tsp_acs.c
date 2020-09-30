/* Author : Thodoris Apostolopoulos */
//*****************************************************************************************
// Travelling Salesman Problem with Ant Colony System Optimization 
//*****************************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N 100
#define X 1000
#define A 5                 // ants used 
#define square(x) ((x)*(x)) 
//*********************************************************************
float cities[N][2] ;        // each city has Cartesian coordinates : cities[N][0] = x and cities[N][1] = y 
int map[X+1][X+1] ;         // map[1001][1001] each city with Cartesian coordinates in the map
float cityDist[N][N] ;      // NxN matrix shows the distances between the cities
float h[N][N] ;             // shows the visibility for each edge : h[N][N] = 1 / cityDist[N][N]
float pherDistr[N][N] ;     // shows the pheremone distribution for each edge
float routeDist[A] ;        // total route distance each A ant/salesman will take
int route[A][N+1] ;         // the tour ant/salesman that will take 
float t[N][N] ;             
//*********************************************************************
// initCities : initialiazes cities and map matrices 
//*********************************************************************
void initCities(){

    int i ;
    int x, y ;
    for(i=0;i<N;i++){

        x = rand() % (X+1) ;        // x range [0,1000]
        y = rand() % (X+1) ;        // y range [0,1000]
        while(map[y][x]) {          // while the city is already chosen 
            x = rand() % (X+1) ;    // takes new random coordinates
            y = rand() % (X+1) ; 
        }
        map[y][x] = 1 ;             // each time a city chosen becomes 1
        cities[i][0] = x ;
        cities[i][1] = y ;
    }
}
//*********************************************************************
// init : create matrices cityDist[N][N], h[N][N], pherDistr[N][N] 
//*********************************************************************
void init(){
  
    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){

            cityDist[i][j] = sqrt(square(cities[i][0] - cities[j][0]) + square(cities[i][1] - cities[j][1])) ;
            h[i][j] = 1 / cityDist[i][j] ;

            if(i != j){
                pherDistr[i][j] = 1 ;
            }
        }
    }
}
//*********************************************************************
// antPath : makes a full path 
//*********************************************************************
void antPath(int start){

    int  i, j, k, next ;               
    float a = 0.5 , b = 0.5 ;           // weights of the equation 
    float prop, max, sum ;              // propability of the next city
                       
    int pass[N] = {0} ;                 // shows the cities been passed by ant/salesman

    int previous = start ;              // the city ant starts from 
    route[start][0] = previous ;        // 1st city starts from
    pass[previous] = 1 ;                // city started from has been passed

    for(i=1;i<N;i++){                   // N-1 paths ant will take before getting back

        max = 0 ;
        //******Applying the propability equation of the ACO for the next cities******
        for(j=0;j<N;j++){               // j cities which remain to be visited when the ant is at city previous

            sum = 0 ;
            for(k=0;k<A;k++){           
                if(pass[k] == 0 && cityDist[previous][k] != 0){
                    sum += pow(pherDistr[previous][k],a) * pow(h[previous][k],b) ;
                }
            }
            if(pass[j] == 0 && cityDist[previous][j] != 0){
                prop = (pow(pherDistr[previous][j],a) * pow(h[previous][j],b)) / sum ;
            }
            else{
                prop = 0 ;
            }
            if(prop > max){
                max = prop ;
                next = j ;
            }
            //printf("propability from city %d to %d %f\n",previous, j, prop) ;
        }
        //******************************************************************************
        //printf("Max propability = %f\nCity next : %d\n\n", max, next) ;
        pass[next] = 1 ;                                                // city passed takes value 1
        routeDist[start] += cityDist[previous][next] ;                  // sum of all cities chosen 
        route[start][i] = next ; 
        previous = next ;
    }
    // last path ant returns to start city
    routeDist[start] += cityDist[previous][start] ;
    route[start][N] = start ;
}
//*********************************************************************
// estimatePherem : calculates the amount of pheremone for every edge
//*********************************************************************
void estimatePherem(){

    float p = 0.5 ;
    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){
            t[i][j] = 0 ;
        }
    }
    for(int i=0;i<A;i++){
        for(int k=1;k<N+1;k++){
            t[route[i][k-1]][route[i][k]] += 1 / routeDist[i] ;
            t[route[i][k]][route[i][k-1]] += 1 / routeDist[i] ;
        }
    }
    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){
            pherDistr[i][j] = (1 - p) * pherDistr[i][j] + t[i][j] ;
        }
    }

}
//*********************************************************************
// printMatrix : prints the arrays
//*********************************************************************
void printMatrix(){

    for(int i=0;i<N;i++){
        for(int j=0;j<2;j++){
            printf("%f  ",cities[i][j]) ;
        }puts("");
    }
    puts("");
    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){
            printf("%f  ",h[i][j]) ;
        }puts("");
    }
    puts("");
    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){
            printf("%f  ",pherDistr[i][j]) ;
        }puts("");
    }
    puts("");
}
//*********************************************************************
// Main 
//*********************************************************************
int main () {

    initCities() ; 
    init() ;
    //printMatrix() ;

    float min = 1.0e30 ;
    for(int i=0;i<N;i++){                       // A iterations

        printf("\nIteration %d\n",i+1) ;

        for(int j=0;j<A;j++){                   // A ants to N different cities 
            routeDist[j] = 0 ;
            antPath(j) ;
        }
        for(int j=0;j<A;j++){   
            printf("\nAnt %d\nDistance = %f\n", j+1, routeDist[j]) ;                
            if(routeDist[j] < min) {
                min = routeDist[j] ;
            }
        }
        estimatePherem() ;
    }

    printf("\nMin Path = %f", min) ; 
    return 0 ;
}