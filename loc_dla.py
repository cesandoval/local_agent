import rhinoscriptsyntax as rs
import random
import Rhino

random.seed(0)

class DLA():
    def __init__(self, POS, GUID, max_length, percent_back, LINE=None):
        self.pos = POS
        self.id = GUID
        self.line = LINE
        self.fertile = True
        self.max_length = None
        self.percent_back = percent_back
        
    def aggregate(self, howMany, howFar, otherDlaPts, attractors):
        if self.fertile:
            newDlaPts = []
            for i in range(howMany):
                
                #randomize
                #x = random.random()-0.5
                x = random.randint(-1,1)
                #print x
                #y = random.random()-0.5
                y = random.randint(-1,1)
                
                #z = random.random()-0.5
                z = random.randint(-1,1)
                while [x,y,z] == [0,0,0] :
                    x = random.randint(-1,1)
                    y = random.randint(-1,1)
                    z = random.randint(-1,1)
                    #print x
                #make the random vector
                vec = rs.VectorUnitize([x,y,z])
                #rnd = random.random()
                #print rnd
                
                #adjust it to the attractors
                adjValue = self.attractor(self.pos, attractors, howFar+6)
                #scale it to how far
                vec = rs.VectorScale(vec, (howFar+.3)*adjValue)
                #add it to the coordinate of your pos
                newPos = rs.PointAdd(self.pos,vec)
                
                #make the point in rhino
                newID = rs.AddPoint(newPos)

                #make a line between you and the new random point
                rndBack = random.uniform(0,self.percent_back)
                backVec = rs.VectorScale(vec,-1)
                backVec = rs.VectorScale(backVec, rndBack)
                backPos = rs.PointAdd(newPos,backVec)
                
                # This creates the points
                backID = rs.AddPoint(backPos)
                
                if rs.VectorLength (vec) > self.max_length:
                    # this adds the line in Rhino
                    line = rs.AddLine(self.pos, newPos)
                    
                    #find the closest other point in the aggregation
                    otherPtCoords = []
                    for otherPt in otherDlaPts:
                        otherPtCoords.append(otherPt.pos)
                    index = rs.PointArrayClosestPoint(otherPtCoords, backPos)
                    
                    #calculate the distance between the new point and the closest other point
                    distance = rs.Distance(backPos, otherPtCoords[index])
                    if distance < howFar: 
                        rs.AddLine(backPos, otherPtCoords[index])
                    newDlaPts.append( DLA (backPos , backID, self.max_length, self.percent_back, line))
            self.fertile = False
            return newDlaPts
            
        else:
            return "empty"
            
    def attractor(self, endPt, attrs, thres):
        # Define a new array of Attractors' Coordinates from the Attractors' Identifiers
        attrCoords = attrs
        
        # Calculate the closest Attractor to the Test Point
        closestAttrIndex = rs.PointArrayClosestPoint(attrCoords, endPt)
        closestAttr = attrCoords[closestAttrIndex]
        
        # Calculate the Distance between the Test Point and its closest Attractor
        dist = rs.Distance(endPt, closestAttr)

        # Calculate the Adjustment Amount:
        # If the Distance is less than the Threshold Then
        if dist:
            #The Adjustment Amount will be 1 - ( ( Threshold - Distance ) / Threshold )
            return 1.03-((thres-dist)/thres) # large num to divide = smaller branches 
            
        #Else
        else:
            #The Adjustment Amount will be 1
            return .4 # Max Branch length
            
        # End If
        
    # End Function