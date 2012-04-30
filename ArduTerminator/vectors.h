
#ifndef VECTORS_H
#define VECTORS_H

struct LongPoint
{
	unsigned long x;   // Coordinate axes
	unsigned long y;
        LongPoint();
};

inline LongPoint::LongPoint()
{
  x = 0;
  y = 0;
}

inline LongPoint operator+(const LongPoint& a, const LongPoint& b)
{
  LongPoint result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  return result;
}

inline LongPoint operator-(const LongPoint& a, const LongPoint& b)
{
  LongPoint result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  return result;
}

unsigned long dot(LongPoint A, LongPoint B, LongPoint C){
        LongPoint AB;
        LongPoint BC;
        AB.x = B.x-A.x;
        AB.y = B.y-A.y;
        BC.x = C.x-B.x;
        BC.y = C.y-B.y;
        unsigned long dot = AB.x * BC.x + AB.y * BC.y;
        return dot;
    }
    
  unsigned long cross(LongPoint A, LongPoint B, LongPoint C){
      
        LongPoint AB;
        LongPoint AC;
        AB.x = B.x-A.x;
        AB.y = B.y-A.y;
        AC.x = C.x-A.x;
        AC.y = C.y-A.y;
        unsigned long cross = AB.x * AC.y - AB.y * AC.x;
        return cross;
    }
    
      //Compute the distance from A to B
    unsigned long distanc(LongPoint A, LongPoint B){
        unsigned long d1 = A.x - B.x;
        unsigned long d2 = A.y - B.y;
        // above two lines could also be written LongPoint d = A-B;  ( with operator overloading ) 
        return sqrt(d1*d1+d2*d2);
    }
    
    //Compute the distance from segment AB to C
    //if isSegment is true, AB is a segment, not a line.
    unsigned long linePointDist(LongPoint A, LongPoint B, LongPoint C){
       //int isSegment = 1;
        unsigned long dist = cross(A,B,C) / distanc(A,B);
       // if(isSegment){
            int dot1 = dot(A,B,C);
            if(dot1 > 0)return distanc(B,C);
            int dot2 = dot(B,A,C);
            if(dot2 > 0)return distanc(A,C);
       // }
       // poor mans abs function for unsigned longs: 
       if (dist < 0 ) return dist*-1.0;
       return dist;
      
    }
    
#endif

