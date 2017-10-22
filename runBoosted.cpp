/*
 * robotRun.c
 *
 *  Created on: Aug 13, 2014
 *      Author: xmsun
 */
#include<stdio.h>
#include<math.h>
#include<cstdlib>



// Define a Point structure
struct Point {

 double x;
 double y;
 } ;

 // Define a Robot structure
struct Robot{

  int ID;
  double x;
  double y;
  double drct;
  };


  //List structure, it can be neighborList
  struct  ListofPoints{
	  struct Point *List;
	  int size;
  };
  //
struct Obstacle{
	  int id;
	  struct ListofPoints vertices;
	  struct ListofPoints originalVertices;
	  struct Point interiorPoint; //used for sight blocking or normal vector side determination
	  double smallestX;
	  double smallestY;
	  double largestX; //Usually used for scanning, give a sense of the the size of the shape
	  double largestY;
  } ;

  //List structure, it can be obstacleList
struct ListofObstacles{
  	  struct Obstacle *List;
  	  int size;
    } ;
    //

#define sensingDecayFactor  0.12
#define sensingCutoffRange  80
#define robotMaxDistance 0.5
#define maxNumRobot 4
#define maxNumObstacle 4
#define infinitelyLarge 99999;
#define speed 0.5;


#define MIN(x,y) (((x)<(y))? (x):(y))
#define MAX(x,y) (((x)>(y))? (x):(y))

  // To find the distance between two points X and Y
 double Dist(struct Point X, struct Point Y)
 {
	 double x=X.x-Y.x;
	 double y=X.y-Y.y;
	 return sqrt(x*x+y*y);
 }

 double max(double x, double y){
	 if(x>y) return x;
	 else return y;
 }

 double min(double x, double y){
 	 if(x>y) return y;
 	 else return x;
 }

 double dot(struct Point  a, struct Point  b)
 {
         return a.x * b.x + a.y * b.y;
 }

 struct Point minus(struct Point a, struct Point b)
 {
	     struct Point newP;
         newP.x=a.x-b.x;
         newP.y=a.y-b.y;
         return newP;
 }

 struct Point Norm1minus(struct Point a, struct Point b)
  {
	 	  struct Point newP;
          double tx,ty;
          tx=a.x-b.x;
          ty=a.y-b.y;
          newP.x=-ty;
          newP.y=tx;
          return newP;
  }

 struct Point Norm2minus(struct Point a, struct Point b)
   {
	 	 struct Point newP;
           double tx,ty;
           tx=a.x-b.x;
           ty=a.y-b.y;
           newP.x=ty;
           newP.y=-tx;
           return newP;

   }

 const int OnSegment(struct Point a, struct Point b, struct Point c)
            {
                if ((min(a.x, b.x) <= c.x && c.x <= max(a.x, b.x)) && (min(a.y, b.y) <= c.y && c.y <= max(a.y, b.y)))
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }



struct Point plus(struct Point a, struct Point  b)
{
	struct Point newP;
    newP.x=a.x+b.x;
    newP.y=a.y+b.y;
    return newP;
}

struct Point divide(struct Point a, double b)
{
	struct Point newP;
	newP.x=a.x/b;
	newP.y=a.y/b;
    return newP;
}

struct Point ExtendToInfinite(struct Point startPoint, struct Point middlePoint) {
	        /*return newP struct Point(middlePoint.x +
	                          infinitelyLarge * (middlePoint.x - startPoint.x),
	                          middlePoint.y +
	                          infinitelyLarge * (middlePoint.y - startPoint.y));*/
			struct Point newP;
	        newP.x=middlePoint.x +
                    99999 * (middlePoint.x - startPoint.x);
	        newP.y=middlePoint.y +
                    99999 * (middlePoint.y - startPoint.y);
	        return newP;
}

const int HasIntersection(struct Point p1, struct Point p2, struct Point p3, struct Point p4) //using final might help performance, I am not sure
             {

                 double d1 = (p1.x - p3.x) * (p4.y - p3.y) - (p1.y - p3.y) * (p4.x - p3.x);
                 double d2 = (p2.x - p3.x) * (p4.y - p3.y) - (p2.y - p3.y) * (p4.x - p3.x);
                 double d3 = (p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x);
                 double d4 = (p4.x - p1.x) * (p2.y - p1.y) - (p4.y - p1.y) * (p2.x - p1.x);

                 if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
                 {
                     return 1;
                 }
                 else if ((d1 == 0) && OnSegment(p3, p4, p1))
                 {
                     return 1;
                 }
                 else if ((d2 == 0) && OnSegment(p3, p4, p2))
                 {
                     return 1;
                 }
                 else if ((d3 == 0) && OnSegment(p1, p2, p3))
                 {
                     return 1;
                 }
                 else if ((d4 == 0) && OnSegment(p1, p2, p4))
                 {
                     return 1;
                 }
                 else
                 {
                     return 0;
                 }
             }


int LineOfSightMinusOneVertex(struct Point p1, struct Point p2, struct Point IgnoredVertex, struct Obstacle b)
    {
        
            struct Point previousPoint = b.vertices.List[b.vertices.size - 1];
			int i;

			for (i = 0; i < b.vertices.size; i++)
            {
                if ((b.vertices.List[i].x == IgnoredVertex.x&&b.vertices.List[i].y == IgnoredVertex.y) || (previousPoint.x == IgnoredVertex.x&&previousPoint.y == IgnoredVertex.y))
                {
                    previousPoint = (b.vertices.List[i]);
                }
                else
                {
                    if (HasIntersection(p1, p2, b.vertices.List[i], previousPoint))
                    {
                        return 0;
                    }
                    else
                    {
                        previousPoint = b.vertices.List[i];
                    }
                }
            }
            return 1;
        

    }

 void updateInteriorPoint(struct Obstacle* b)
 	{
 	    struct Point sum;
		int i;
 	    sum.x=0;
 	    sum.y=0;
		

 	    for (i = 0; i < b->vertices.size; i++)
 	    {
 	        sum=plus(sum, b->vertices.List[i]);
 	    }
 	    b->interiorPoint = divide(sum, (double) b->vertices.size);
 	}


struct ListofPoints  getNeighborList(struct Robot localInfo[], int myID)
 {
	 struct Point position ={0, 0};
	 struct Point temp = {0, 0};
	 struct Point tmp[100];
	 struct ListofPoints neighborList;
	 neighborList.List=tmp;
	 static struct Point tempNeighborList[maxNumRobot];

	 int k=0;
	 int i;
	 for(i = 0; i < maxNumRobot; i++)
		  {
			  if (localInfo[i].ID == myID)
			  {
				  position.x = localInfo[i].x;
				  position.y = localInfo[i].y;
				  break;
			  }
		  }
	  if(position.x ==0 && position.y ==0)
	  {
		  printf("My location is not there!");

		  neighborList.size = 0;
	  }
	  else
	  {

		 for( i = 0; i < maxNumRobot; i++)
			  {

				  if (localInfo[i].ID != myID)
				  {
					  temp.x = localInfo[i].x;
					  temp.y = localInfo[i].y;

					 if(Dist(temp, position)< 2*sensingCutoffRange)
					 {
						 tempNeighborList[k].x = temp.x;
						 tempNeighborList[k].y = temp.y;
						 k = k+1;
					 }
				  }
			  }
		 neighborList.size = k;
		 neighborList.List = tempNeighborList;
	  }

	  return neighborList;
 }

 double SensingModelFunction(double distance) //exponentially decreasing with the distance
    {
        //Make sure it is between 0 to 1

         return exp(-sensingDecayFactor * distance);
    }

 double SensingModelDerivative(double distance)
 {

	 return -sensingDecayFactor * exp( -sensingDecayFactor * distance);
 }


// Based on  the obstacle class
 void ListAdd(struct Point sourcePoint, struct ListofPoints *targetList)
   { // add the sourceList after the targetList
  	 int size = targetList->size;

  	 targetList->List[size] = sourcePoint;
  	 targetList->size +=1;
 }

void GetActiveReflexVertices(struct Point observerPosition, struct Obstacle obstacle, struct ListofPoints *listARV)
     {  // input one obstacle
		// Return the listARV;
	 	int verticeSize = obstacle.vertices.size;
		int i;

         for (i = 0; i < verticeSize; i++)
         {
             if (LineOfSightMinusOneVertex(observerPosition, obstacle.vertices.List[i], obstacle.vertices.List[i],obstacle))
             { //the observer can see this vertex
                 if (LineOfSightMinusOneVertex(ExtendToInfinite(observerPosition, obstacle.vertices.List[i]), obstacle.vertices.List[i], obstacle.vertices.List[i],obstacle))
                 {
                     //The line connecting observer and the vertex doesn't cut through the shape
                     //listARV.add(obstacle.vertices.List[i]);
                     ListAdd(obstacle.vertices.List[i],listARV);
                 }
             }
         }
     }


    void updateBoundingBox(Obstacle *b)
    {
		b->smallestX = b->largestX = b->vertices.List[0].x;
        b->smallestY = b->largestY = b->vertices.List[0].y;
        for (int i = 1; i < b->vertices.size; i++)
        {
            if (b->vertices.List[i].x < b->smallestX)
            {
                b->smallestX = b->vertices.List[i].x;
            }
            if (b->vertices.List[i].y < b->smallestY)
            {
                b->smallestY = b->vertices.List[i].y;
            }
            if (b->vertices.List[i].y > b->largestY)
            {
                b->largestY = b->vertices.List[i].y;
            }
            if (b->vertices.List[i].x > b->largestX)
            {
                b->largestX = b->vertices.List[i].x;
            }

        }

	
    }

         const int LineOfSight(struct Point p1, struct Point p2, struct Obstacle b)
             {
				 int i;
                 //bounding box testing;
                 if (p1.x > b.largestX && p2.x >b.largestX)
                 {
                     return 1;
                 }
                 if (p1.y > b.largestY && p2.y > b.largestY)
                 {
                     return 1;
                 }
                 if (p1.x < b.smallestX && p2.x < b.smallestX)
                 {
                     return 1;
                 }
                 if (p1.y < b.smallestY && p2.y < b.smallestY)
                 {
                     return 1;
                 }

                 if (b.originalVertices.List == NULL || b.originalVertices.size == 0)
                 { //It's empty, so it can block nothing.
                      return 1;
                 }
                 else
                 {
                     struct Point previousPoint = b.originalVertices.List[b.originalVertices.size - 1];
                     for (i = 0; i < b.originalVertices.size; i++)
                     {
                         if (HasIntersection(p1, p2, b.originalVertices.List[i], previousPoint))
                         {
                             return 0;
                         }
                         else
                         {
                             previousPoint = b.originalVertices.List[i];
                         }
                     }

                     return 1;
                 }
            }

		 int LineOfSight(struct Point p1, struct Point p2, struct Point direction, struct Obstacle b)
		 {
			         //bounding box testing; seems to be a cut the cost to 60%
        if (p1.x > b.largestX && p2.x > b.largestX)
        {
            return 1;
        }
        if (p1.y > b.largestY && p2.y > b.largestY)
        {
            return 1;
        }
        if (p1.x < b.smallestX && p2.x < b.smallestX)
        {
            return 1;
        }
        if (p1.y < b.smallestY && p2.y < b.smallestY)
        {
            return 1;
        }

         if (b.vertices.size == 0)
         { //It's empty, so it can block nothing.
            return 1;
         }
         else
        {       
        	//TODO: a line can actually intersect with an obstacle in two places. We should return the closest direction
        	//instead of the first one detected.
            Point previousPoint = b.vertices.List[b.vertices.size - 1];
            for (int i = 0; i < b.vertices.size; i++)
            {
                if (HasIntersection(p1, p2, b.vertices.List[i], previousPoint))
                {
                    //direction = Point.minus(vertices.get(i), previousPoint);
                    direction.x = minus(b.vertices.List[i], previousPoint).x;
                    direction.y = minus(b.vertices.List[i], previousPoint).y;
                    return 0;
                }
                else
                {
                    previousPoint = b.vertices.List[i];
                }
            }

            return 1;
        }
    }
		 

     	struct Point GetIntersection(struct Point p1, struct Point p2, struct Point p3, struct Point p4)
     	    {
     		struct Point newP;
     		newP.x=((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p3.x * p4.y - p3.y * p4.x) * (p1.x - p2.x)) /
                     ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y));
     		newP.y=((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p3.x * p4.y - p3.y * p4.x) * (p1.y - p2.y)) /
                     ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y));
     	        //if two lines are parallel, the return value will be NaN and Infinite
     	        //the line segments (p1,p2) and (p3,p4) do not have to intersect. The lines defined by them will intersect if not parallel.
     	    return newP;
     	    }
	int GetImpactPoint(struct Point startPoint, struct Point farawayPoint, struct Point* CurrentBestImpactPoint, int Uninitialized, struct Obstacle b)
    {
        struct Point impactPointCandidate;
        int hasBetterImpactCandidate = 0;
        struct Point previousPoint = b.vertices.List[b.vertices.size - 1];
		int i;
        for (i = 0; i < b.vertices.size; i++)
        {
            if (HasIntersection(startPoint, farawayPoint, b.vertices.List[i], previousPoint))
            {
                impactPointCandidate = GetIntersection(startPoint, farawayPoint, b.vertices.List[i], previousPoint);
                if (Uninitialized)
                {
                    CurrentBestImpactPoint->x = impactPointCandidate.x;
                    CurrentBestImpactPoint->y = impactPointCandidate.y;
                    hasBetterImpactCandidate = 1;
                    Uninitialized = 0;  //TODO: changing the value of the method parameter, is the change on Uninitialized passed back to the caller?
                }
                else
                {
                    if (Dist(impactPointCandidate, startPoint) < Dist(*CurrentBestImpactPoint, startPoint))
                    {
                        CurrentBestImpactPoint->x = impactPointCandidate.x;
                        CurrentBestImpactPoint->y = impactPointCandidate.y;
                        hasBetterImpactCandidate = 1;
                    }
                }
            }
            //else
            // {
            previousPoint = b.vertices.List[i];
            // }
        }
        return hasBetterImpactCandidate;
    }

	int isPointInLOS(struct Point position, struct Point samplePoint, struct ListofObstacles obstacles) //in line of sight, not considering FOV, only obstacles
    {
    	 for (int i1 = 0; i1 < obstacles.size; i1++)
         {
             if (!LineOfSight(position, samplePoint, obstacles.List[i1]))
             {
                 return 0;
             }
         }
    	 return 1;
    }
	    int is_point_visible(struct Point sample_point, struct Point position, struct Obstacle boundary, struct ListofObstacles obstacles) //Notice that a point coincide with robot position is NOT visible
    {
    	double distance = Dist(sample_point, position);
        
        if (distance > sensingCutoffRange || distance <= 0) //to make sure dist>0 is necessary. SamplePoint might coincide with robot position. dist=0 will cause divide by zero
        {	
        	return 0;
        }
    	
        //boundary block all sensing. This test can be disabled if nodes can not stay out of boundary and boundary is always a rectangle.  
        if (!LineOfSight(position, sample_point, boundary))
			
			return 0;
        
        if(!isPointInLOS(position, sample_point, obstacles))
        {
        	return 0;
        }
              
    	return 1;
    }
    
		int is_point_visible(struct Point sample_point, struct Point position, struct Obstacle boundary)
		{
		
			double distance = Dist(sample_point, position);
        
        if (distance > sensingCutoffRange || distance <= 0) //to make sure dist>0 is necessary. SamplePoint might coincide with robot position. dist=0 will cause divide by zero
        {	
        	return 0;
        }
    	
        //boundary block all sensing. This test can be disabled if nodes can not stay out of boundary and boundary is always a rectangle.  
        if (!LineOfSight(position, sample_point, boundary))
		{
			return 0;

		}
		return 1;
        
		}

		 double NeighborEffects(struct Point samplePoint, struct ListofPoints neighborList,struct Obstacle boundary, struct ListofObstacles obstacles)
 {
	 	 double neighborEffects = 1;
	 	 int alpha;
		 int i1;
	 	 for(i1=0; i1 < neighborList.size; i1++)
	 	 {
			 alpha = 1; // Error before. alpha needs to be initialized for All neighbors.

			 if(!is_point_visible(samplePoint, neighborList.List[i1] , boundary, obstacles))
	 		 {
	 			 alpha = 0;
	 		 }

	 		 neighborEffects *= (1 - alpha * SensingModelFunction(Dist(samplePoint, neighborList.List[i1])));
	 	 }

	 	 return neighborEffects;
 }


 struct Point RobotAlgorithm(struct Robot localInfo[], int myID, int boostType)
//  localInfo[] is a list of neighbor robots and itself.
		{
	  	  	  //Constants of mission space
	 	 	  
	 	 	 
	  	  	  //Physical properties of a robot
	  	  	  //Control parameters
	  	  	  double delta = 1; // delta is the surface integral increment parameter
	  	  	  // Compute the next position
	  	  	  double dFdx = 0;
	  	  	  double dFdy = 0;
	  	  	  // Compute the obstacle line integral
	  	  	  double dFdx2 = 0;
	  	  	  double dFdy2 = 0;
			  double heading;
	  	  	  struct Point tempPosition = {0, 0};
	  	  	  // Identify the current position and neighbor list
	  	  	  struct Point position = {0, 0};
	  	  	  // No boosting function, no boundary are considered now. But we use boosted defination about weights and so on.
	  	  	  struct Point samplePoint = {-1,-1};
	  	  	  double dist;
	  	  	  double neighborEffects;
	  	  	  double w1=0;
	  	  	  double Alpha1 = 1;
	  	      double Beta1 = 0;
	  	      double alpha;
	  	      double magnitude=1;
	  	  	  struct ListofPoints neighborList;
			  struct ListofObstacles obstacles;
			  struct ListofPoints activeReflexVertices;
			  int runIntoObstacle = 0;
			  struct Point direction = {0, 0};
			  int i;
			  double probability;
			  int PowerGain = 500;
			  int Power = 3;


	  	  	  // Obstacles related
	 	 	 static struct Obstacle obstacleList[maxNumObstacle];
	 	  	 struct Point vertice[maxNumObstacle][4];
			 struct Point tmp[100];
			 struct Obstacle boundary;
			 //boundary order matters
			  boundary.vertices.List=tmp;
			  boundary.largestX=45;
	 	 	  boundary.largestY=50;
			  boundary.vertices.List[0].x = 0;
			  boundary.vertices.List[0].y = 0;
			  boundary.vertices.List[1].x = boundary.largestX;
			  boundary.vertices.List[1].y = 0;
			  boundary.vertices.List[2].x = boundary.largestX;
			  boundary.vertices.List[2].y = boundary.largestY;
			  boundary.vertices.List[3].x = 0;
			  boundary.vertices.List[3].y = boundary.largestY;
			  boundary.vertices.size = 4;
			  boundary.originalVertices = boundary.vertices;
			  updateBoundingBox(&boundary);
				  

			 vertice[0][0].x=30;
			 vertice[0][0].y=0;
	 	 	 vertice[0][1].x=30;
			 vertice[0][1].y=40;
	 	 	 vertice[0][2].x=34;
			 vertice[0][2].y=40;
	 	 	 vertice[0][3].x=34;
			 vertice[0][3].y=0;
	 	 	 vertice[1][0].x=70;
			 vertice[1][0].y=0;
	 	 	 vertice[1][1].x=70;
			 vertice[1][1].y=40;
	 	 	 vertice[1][2].x=74;
			 vertice[1][2].y=40;
	 	 	 vertice[1][3].x=74;
			 vertice[1][3].y=0;
	 	 	 vertice[2][0].x=14;
			 vertice[2][0].y=52;
	 	 	 vertice[2][1].x=14;
			 vertice[2][1].y=100;
	 	 	 vertice[2][2].x=16;
			 vertice[2][2].y=100;
	 	 	 vertice[2][3].x=16;
			 vertice[2][3].y=52;
	 	 	 vertice[3][0].x=54;
			 vertice[3][0].y=52;
	 	 	 vertice[3][1].x=54;
			 vertice[3][1].y=100;
	 	 	 vertice[3][2].x=56;
			 vertice[3][2].y=100;
	 	 	 vertice[3][3].x=56;
			 vertice[3][3].y=52;

			 for(int i=0;i<4;i++)
			 {
				 for(int j=0;j<4;j++)
				 {
					 vertice[i][j].x=vertice[i][j].x * 0.5;
					 vertice[i][j].y=vertice[i][j].y * 0.5;
				 }
			 }

	 	 	 for( i=0; i < maxNumObstacle; i++)
	 	 	 {
	 	 		 obstacleList[i].vertices.List = &vertice[i][0];
	 	 		 obstacleList[i].vertices.size = 4;
	 	 		 obstacleList[i].originalVertices.List=&vertice[i][0];
	 	 		 obstacleList[i].originalVertices.size=4;
	 	 		 obstacleList[i].id = i;
	 	 		 updateInteriorPoint(&obstacleList[i]);
				 updateBoundingBox(&obstacleList[i]);

	 	 	 }

	  	  	 
		  	  obstacles.List= obstacleList;
		  	  obstacles.size = maxNumObstacle;

	  	  	 
	  	  	  activeReflexVertices.size=0;
			  struct Point p[100];
			  p[0].x=-1;
			  p[0].y=-1;
	  	  	  activeReflexVertices.List = p;



	  	  	  //int i;
	  	  	  for( i = 0; i < maxNumRobot; i++)
	  	  	  {
//				  printf("localInfo[%d].ID: %d\n", i,localInfo[i].ID);
//				  printf("myID is %d \n", myID);
	  	  		  if (localInfo[i].ID == myID)
	  	  		  {
	  	  			  position.x = localInfo[i].x;
	  	  			  position.y = localInfo[i].y;
	  	  			  break;
	  	  		  }
	  	  	  }
	  	  	  if(position.x ==0 && position.y ==0)
	  	  	  {
	  	  		  printf("My location is not there!");
	  	  	  }
	  	  	  else
	  	  	  {
	  	  		neighborList = getNeighborList(localInfo, myID);
	  	  	  }


	  	  	  double sample_upper_bound_x = min(position.x + sensingCutoffRange, boundary.largestX);
	  	  	  double sample_upper_bound_y = min(position.y + sensingCutoffRange, boundary.largestY);

	  	  	  for(double horizontalSamplePoint = max(position.x-sensingCutoffRange, 0.01);horizontalSamplePoint<=sample_upper_bound_x; horizontalSamplePoint+=delta)
	  	  	  {	
				  for (double verticalSamplePoint = max(position.y-sensingCutoffRange, 0.01); verticalSamplePoint<=sample_upper_bound_y; verticalSamplePoint+=delta)
	            {
	                samplePoint.x = horizontalSamplePoint;
	                samplePoint.y = verticalSamplePoint;
	                dist = Dist(samplePoint, position);

					if(is_point_visible(samplePoint,position,boundary, obstacles))
					{
						alpha = 1;
						
					}

					else
	                {
	                	alpha = 0;
						//printf("samplePoint is: %f %f, position is %f, %f, distance is %f\n",samplePoint.x,samplePoint.y, position.x, position.y,dist);
						
	                }


					if(alpha > 0)
					{
						neighborEffects = NeighborEffects(samplePoint,neighborList,boundary,obstacles);
						//printf("samplePoint is %f, %f position is %f, %f neighborEffects is %f\n", samplePoint.x, samplePoint.y, position.x, position.y, neighborEffects);
						w1 =  alpha * (-SensingModelDerivative(dist)) * neighborEffects;

						if(boostType == 1)
						{
							
							probability =1-neighborEffects*(1-SensingModelFunction(dist));
							Alpha1 = PowerGain / pow(probability,Power);
						}

	              		dFdx += (Alpha1 * w1 + Beta1) * ( horizontalSamplePoint - position.x ) / dist * delta * delta;
						dFdy += (Alpha1 * w1 + Beta1) * ( verticalSamplePoint - position.y) / dist * delta * delta;

					}
			  }
	  	  	 };
			 // printf("myID is %d \n", myID);
			  //printf("Before add: dFdx: %f, dFdy: %f\n", dFdx, dFdy);
	  	  	  // Calculating the line integral part
	  	  	  // 以下部分计算dFdx2，dFdy2.

	          dFdx2 = 0;
	          dFdy2 = 0;

	          //Assuming the boundary is convex


	          for (int i2 = 0; i2 < obstacles.size; i2++)
	          {
	              GetActiveReflexVertices(position, obstacles.List[i2], &activeReflexVertices); //must clear activeReflexVertices before calling this again
	              // Todo: nothing in activeReflexVertices
	              for (int i3 = 0; i3 < activeReflexVertices.size; i3++)
	              {
	                  double D = Dist(activeReflexVertices.List[i3], position);
	                  if (D < sensingCutoffRange)
	                  {
	                      int ARVBlocked = 0; //test if an ARV is blocked by other FG
	                      if (!(LineOfSight(position, activeReflexVertices.List[i3],boundary)))
	                      {
	                          ARVBlocked = 1;
	                      }
	                      else
	                      {
	                          for (int i4 = 0; i4 < obstacles.size; i4++)
	                          {
	                              if (i2 != i4)
	                              {
	                                  if (!(LineOfSight(position, activeReflexVertices.List[i3],obstacles.List[i4])))
	                                  {
	                                      ARVBlocked = 1;
	                                      break;
	                                  }
	                              }
	                          }
	                      }

	                      if (!ARVBlocked)
	                      {
	                          //SYNC
	                          struct Point impactPoint; //it should be filled with useful values before used by algorithm
	                          impactPoint.x=-1;
	                          impactPoint.y=-1;
	                          GetImpactPoint(activeReflexVertices.List[i3], ExtendToInfinite(position, activeReflexVertices.List[i3]),
	                                                  &impactPoint, 1, boundary);

	                          for (int i5 = 0; i5 < obstacles.size; i5++)
	                          {
	                              if (i5 != i2)
	                              {
									  // Debugged Error; we need to return the impactPoint;
	                                  GetImpactPoint(activeReflexVertices.List[i3],
	                                          ExtendToInfinite(position, activeReflexVertices.List[i3]), &impactPoint, 0, obstacles.List[i5]);
	                              }
	                          }
	                          double d = Dist(impactPoint, activeReflexVertices.List[i3]);
	                          //System.out.println("anchor:"+activeReflexVertices.get(i3).x+","+activeReflexVertices.get(i3).y);

	                          double SinTheta = abs(position.y - activeReflexVertices.List[i3].y) / D;
	                          double CosTheta = abs(position.x - activeReflexVertices.List[i3].x) / D;

	                          struct Point pointingOutsideNorm;
	                          double sign = 1;
	                          if (dot(Norm1minus(position, activeReflexVertices.List[i3]),
	                                         (minus(obstacles.List[i2].interiorPoint, activeReflexVertices.List[i3]))) > 0)
	                          {
	                              pointingOutsideNorm = Norm2minus(position, activeReflexVertices.List[i3]);
	                          }
	                          else
	                          {
	                              pointingOutsideNorm = Norm1minus(position, activeReflexVertices.List[i3]);
	                          }
	                          if (pointingOutsideNorm.x < 0)
	                          {
	                              sign = -1;
	                          }

	                          //Line numerical integration
	                          double alpha1 = 0;
	                          double integralSum = 0;
	                          double delta1 = 0.2; //was 0.05
	                          double integrationUpperBound = d;



	                          if ((D + d) > sensingCutoffRange)
	                          {
	                              integrationUpperBound = sensingCutoffRange - D;
	                          }

	                          struct Point integralPoint;
	                          for (double r = delta1 / 2; r <= integrationUpperBound; r += delta1)
	                          {
	                              //struct Point integralPoint = (impactPoint-activeReflexVertices.get(i3))*r/d+activeReflexVertices.get(i3);
	                              // integralPoint = struct Point.plus(Point.divide(Point.product(Point.minus(impactPoint,
	                              //        activeReflexVertices.get(i3)), r), d), activeReflexVertices.get(i3));
	                              integralPoint.x = (impactPoint.x - activeReflexVertices.List[i3].x) * r / d + activeReflexVertices.List[i3].x;
	                              integralPoint.y = (impactPoint.y - activeReflexVertices.List[i3].y) * r / d + activeReflexVertices.List[i3].y;



	                              //boost low detection area


	  //                           else // no boost
	                             
	                              double Phat, Phat0;

	                              double Alpha2;
	                              double Beta2;
	                                      double w2;
	                                  	neighborEffects = NeighborEffects(integralPoint,neighborList,boundary,obstacles);
	                              		Phat0 = 1 - neighborEffects * (1 - SensingModelFunction(D + r));
	                              		Phat = 1 - neighborEffects * (1 - alpha1 * SensingModelFunction(D + r));
	                              		w2 =  1 * (Phat0-Phat);
	                              		Alpha2 = 1;
	                              		Beta2 = 0;
	                              		integralSum += delta1 * r * (Alpha2 * w2 + Beta2);
	                           }
	                          dFdx2 += sign * SinTheta / D * integralSum;

	                          if (pointingOutsideNorm.y < 0)
	                          {
	                              sign = -1;
	                          }
	                          else
	                          {
	                              sign = 1;
	                          }
	                          //dFdy2+=sign*CosTheta/D*(-d/sensingDecayFactor*SensingModelFunction(d+D)-1/sensingDecayFactor/sensingDecayFactor*(SensingModelFunction(d+D)-SensingModelFunction(D)));

	                          dFdy2 += sign * CosTheta / D * integralSum;

	                      }
	                  }
	              }

	              //activeReflexVertices.clear(); //this container only holds ARV from one obstacle, so when we move to another obstacle, we need to empty it first.
	              	  for(int i=0;i<activeReflexVertices.size;i++){
	              		activeReflexVertices.List[i].x=0;
	              		activeReflexVertices.List[i].y=0;
	              	  }
	              	activeReflexVertices.size=0;
	              }

	          	 dFdx = dFdx + dFdx2;
	  			 dFdy = dFdy + dFdy2;



	          // If we add a term ||si-sj|| in H(s), where j is in i's neighbor set,
	          // the derivative will add (six-sjx)/||si-sj||.

			  printf("myID is %d \n", myID);
			  printf("dFdx: %f, dFdy: %f\n", dFdx, dFdy);

	  	  	  magnitude = sqrt(dFdx*dFdx + dFdy*dFdy);
			  heading = atan2(dFdy, dFdx);
	  	  	  tempPosition.x = position.x + dFdx/magnitude * robotMaxDistance;
	  	  	  tempPosition.y = position.y + dFdy/magnitude * robotMaxDistance;

			  //detect if a robot runs into boundary or an obstacle
			  //obstacle_sliding(point2 tempPosition)
			  //test if robot runs into boundary 		    
		    if(tempPosition.x>boundary.largestX)
		    {
		    	tempPosition.x = boundary.largestX-0.001;
		    }
		    else if(tempPosition.x<boundary.smallestX)
		    {
		    	tempPosition.x = boundary.smallestX+0.001;
		    }
		    
		    if(tempPosition.y>boundary.largestY)
		    {
		    	tempPosition.y = boundary.largestY-0.001;
		    }
		    else if(tempPosition.y<boundary.smallestY)
		    {
		    	tempPosition.y = boundary.smallestY+0.001;
		    }	 

		    //if not into boundary, test if it runs into any obstacles, if it does, it will stop for one turn but the heading will be set properly   
		    for (int i1 = 0; (i1 < obstacles.size) && (!runIntoObstacle); i1++)
		    {
		    	//TODO: there is a bug: if a node cross two boundaries of an obstacle, the direction of the
		    	//first one tested will be returned, not the closest's, a workaround is to describe obstacles
		    	//starting from the lower right corner.
		    	//TODO: another bug is that if a node cross boundaries of two obstacles
		        if (!(LineOfSight(position, tempPosition,direction, obstacles.List[i1])))
		        {
		            if (dot( minus(tempPosition, position), direction) >= 0)
		            {
		                heading = atan2(direction.y, direction.x);                   
		            }
		            else
		            {
		                heading = atan2( -direction.y, -direction.x);
		            }
		            runIntoObstacle = 1;
		            tempPosition = position; 
					tempPosition.x +=  cos(heading)*speed;
					tempPosition.y +=  sin(heading)*speed;
		           
		            
		            if(isPointInLOS(position,tempPosition, obstacles))
		            {
		            	 position = tempPosition;
		            }
		            
		            //position.shift(speed * Math.cos(heading) * updateInterval, speed * Math.sin(heading) * updateInterval);	              
		            return position;
		        }
		    }
		    
		    if (runIntoObstacle == 0)
		    {
		        position = tempPosition;
		    }

			return position;
		}




 int main()
  {
	  int myID = 1;
	  struct ListofPoints neighborList;
//	  neighborList.size = 0;
//	  neighborList.List = &(Point){0,0};
	  struct Robot LocalInfo[maxNumRobot];
	  struct Point tempPosition;
	  int boostType = 0;
	  for(int i=0; i<maxNumRobot;i++)
	  {
		  // Initialize Robots list;
		  LocalInfo[i].drct=0;
		  LocalInfo[i].ID=i+1;
		  LocalInfo[i].x=2*i+1;
		  LocalInfo[i].y=2*i+1;
	  }


	  
	  for(int i=0; i<300; i++)
	  {
		  for(int j=1; j<maxNumRobot+1; j++)
		  {
			  myID = j;
			  tempPosition = RobotAlgorithm(LocalInfo, myID, boostType);
			  LocalInfo[j-1].x = tempPosition.x;
			  LocalInfo[j-1].y = tempPosition.y;
			  printf("ID: %d next position: x is %f, y is %f\n", myID, tempPosition.x, tempPosition.y);

		  }

	   }
	  system("pause"); 
		  boostType = 1;
	  for(int i=0; i<200; i++)
	 {	  for(int j=1; j<maxNumRobot+1; j++)
		  {
			  myID = j;
			  tempPosition = RobotAlgorithm(LocalInfo, myID, boostType);
			  LocalInfo[j-1].x = tempPosition.x;
			  LocalInfo[j-1].y = tempPosition.y;
			  printf("After boosted ID: %d next position: x is %f, y is %f\n", myID, tempPosition.x, tempPosition.y);

		  }
	  }
	 system("pause");  
		boostType = 0;
	  	  for(int i=0; i<200; i++)
	  {
		  for(int j=1; j<maxNumRobot+1; j++)
		  {
			  myID = j;
			  tempPosition = RobotAlgorithm(LocalInfo, myID, boostType);
			  LocalInfo[j-1].x = tempPosition.x;
			  LocalInfo[j-1].y = tempPosition.y;
			  printf("ID: %d next position: x is %f, y is %f\n", myID, tempPosition.x, tempPosition.y);

		  }

	   }
	  system("pause");
	  
	  return 0;
  }


