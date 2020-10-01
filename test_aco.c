/* Thodoris Apostolopoulos */
//*****************************************************************************************
// Travelling Salesman Problem with Ant Colony Optimization 
//*****************************************************************************************
// Test based on paper : Ants = 5, Cities = 5, Iterations = 5
// Cities Distances 
//    A  B  C  D  E 
// A {0, 3, 6, 2, 3}
// B {3, 0, 5, 2, 3}
// C {6, 5, 0, 6, 4}
// D {2, 2, 6, 0, 6}
// E {3, 3, 4, 6, 0}
//*****************************************************************************************
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define N 5                 // N cities, N ants
#define square(x) ((x)*(x)) 
//*********************************************************************
float cityDist[N][N] = { {0, 3, 6, 2, 3}, {3, 0, 5, 2, 3}, {6, 5, 0, 6, 4}, {2, 2, 6, 0, 6}, {3, 3, 4, 6, 0} } ; 
float h[N][N] ;             // shows the visibility for each edge : h[N][N] = 1 / cityDist[N][N]
float pherDistr[N][N] ;     // shows the pheremone distribution for each edge
float routeDist[N] ;        // total route distance each N ant/salesman will take
int route[N][N+1] ;         // the tour ant/salesman that will take 
//*********************************************************************
// init : create matrices h[N][N], pherDistr[N][N] 
//*********************************************************************
void init(){

    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){

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
    float a = 0.7, b = 0.7 ;            // weights of the equation 
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
            for(k=0;k<N;k++){           
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
        routeDist[start] += cityDist[previous][next] ;                  // sum of the route ant takes 
        route[start][i] = next ;                                        
        previous = next ;
    }
    // Last path ant returns to the city it started from
    routeDist[start] += cityDist[previous][start] ;
    route[start][N] = start ;
}
//*********************************************************************
// estimatePherem : calculates the amount of pheremone for every edge
//*********************************************************************
void estimatePherem(){

    float t[N][N] = {0} ;
    float p = 0.5 ;

    for(int i=0;i<N;i++){
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
        for(int j=0;j<N;j++){
            printf("%f  ",h[i][j]) ;
        }puts("");
    }puts("");

    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){
            printf("%f  ",pherDistr[i][j]) ;
        }puts("");
    }puts("");
}
//*********************************************************************
// Main 
//*********************************************************************
int main () {
 
    init() ;
    printMatrix() ;

    int i, j ;
    float min = 1.0e30 ;

    for (i=0;i<N;i++){                          // N = 5 iterations

        printf("\nIteration %d\n",i+1) ;

        for(j=0;j<N;j++){                       // N = 5 Ants put to N = 5 Cities

            routeDist[j] = 0 ;                   
            antPath(j) ;
            for(int k=0;k<N+1;k++){
                printf("%d-->",route[j][k]) ;   // prints the route each ant took
            }puts("");
        }puts("");

        for(j=0;j<N;j++){

            printf("\nAnt %d\nDistance = %f\n", j+1, routeDist[j]) ;
            if(routeDist[j] < min) {
                min = routeDist[j] ;
            }
        }
        puts("\n");

        estimatePherem() ;
        
        for(int i=0;i<N;i++){
            for(int j=0;j<N;j++){
                printf("%f  ",pherDistr[i][j]) ;    // prints the new updated pheremone distribution for each edge
            }puts("");
        }puts("");
    }

    printf("Min Path = %f\n",min) ;
    return 0 ;
}