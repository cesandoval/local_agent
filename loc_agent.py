from decodes.core import *
from decodes.core import dc_base, dc_interval, dc_vec, dc_point  #here we may only import modules that have been loaded before this one.    see core/__init__.py for proper order
import math

class QuadAgent():
    def __init__ (self, capacity, bounds):
        """ QuadTree constructor.
        
            :param capacity: Total number of points to contain.
            :type capacity: int
            :param bounds: Bounds.
            :type bounds: Bounds
            :result: QuadTree object.
            :rtype: QuadTree
        
        """
        self.cap = capacity
        self.bnd = bounds
        self._pts = []
        
    @property
    def has_children(self):
        """ Returns True if QuadTree has children.
        
            :result: Boolean value.
            :rtype: bool
        
        """
        if hasattr(self,'children'):return True 
        return False
        
    @property
    def pts(self):
        """ Recursively returns all the points in this QuadTree.
        
            :result: List of Points.
            :rtype: [Point]
        """
        ret_pts = []
        if not self.has_children :
            ret_pts = [Point(pt) for pt in self._pts]
        else :
            for child in self.children: ret_pts.extend(child.pts)
        return ret_pts

    def append(self, pt) :
        """ Appends the given point to the points in this QuadTree.
        
            :param pt: Point
            :type pt: Point
            :result: Boolean Value
            :rtype: bool
        
        """
        if not self.contains(pt) : return False
        if not self.has_children :
            if len(Point.cull_duplicates([p.pos for p in self._pts])) < self.cap:
                self._pts.append(pt)
                return True
            else :
                self._divide()
        
        if self.has_children :
            for child in self.children:
                if child.append(pt) : return True
            return False
        else:
            raise("quadtree.append()... how did i get here?")
        
    def _divide(self) :
        """ Divides self into sub regions. Starts at bottom left and moves clockwise.
        
            :result: Boolean Value
            :rtype: bool
        """
        if self.has_children: return False
        
        sub_bnds = self.bnd//2
        self.children = [QuadAgent(self.cap,sub_bnd) for sub_bnd in sub_bnds]

        for pt in self._pts : 
            accepted = False
            for child in self.children:
                if child.append(pt) : 
                    accepted = True
                    break
            if not accepted : "no child accepted this point!"
            
        self._pts = None
        return True
    
    def contains(self,pt):
        """ Returns true if given point is in this QuadTree.
        
            :param pt: Point to test for containment.
            :type pt: Point
            :result: Boolean Value.
            :rtype: bool
            
        """
        if self.has_children:
            return any([child.contains(pt) for child in self.children])
        else:
            return pt in self.bnd
            
    def pts_in_bounds(self,bounds):
        """ Finds all points that fall within a given bounds.
        
            :param bounds: bounds
            :type bounds: Bounds
            :result: List of Points in Bounds.
            :rtype: [Point]
            
        """
        if not self.bnd.overlaps(bounds) : return []
        ret_pts = []
        if not self.has_children :
            for pt in self._pts :
                if pt in bounds :  ret_pts.append(pt)
        else :
            for child in self.children: ret_pts.extend(child.pts_in_bounds(bounds))
        return ret_pts
        
    @staticmethod
    def encompass(capacity = 4, pts = [Point()]):
        """ Returns a Bounds that encompasses the given points.
        
            :param capacity: Capacity of points within the Bounds.
            :type capacity: int
            :param pts: List of Points.
            :type pts: [Point]
            :result: Bounds encompassing the given points.
            :rtype: Bounds
        
        """
    
        q = QuadAgent(capacity, Bounds.encompass(pts))
        
        for p in pts : 
            print p
            q.append(p)
        return q


'''
Create a Bird class with flocking behavior

members:
pos (Point) The location of the Bird object
vel (Vec) The velocity vector of the Bird object
max_val (float) The maximum velocity allowed for this Bird
acl (Vec) The acceleration vector of the Bird object
max_frc (float) The maximum force that can be exerted on the Bird
rov (float) Range of vision is the range to which this bird can see other Birds
roc (float) Range of comfort is the range of comfort with nearby Birds
hist_pos (list of Point) The past positions of the Bird
hist_vel (list of Vec) The past velocities of the Bird
hist_acl (list of acl) The past acceleration vectors of the Bird

methods:
step (None) Update Bird's position, velocity and acceleration and record history
steer (Vec) Steer the Bird by changing its accelaration vectors
scan ([Birds]) Scan for Birds that are within the range of vision
in_range([Birds]) Select Birds that are within a certain distance
seek (None) Steer Bird toward a goal
flee (None) Steer Bird away from a goal
align (None) Steer Bird to align with neighboring Birds
clear_history (None) Initialize the Bird's history
record (None) Add the Bird's current status to the Bird's history
'''

"""
A simple Bird class is defined with position, velocity, acceleration vectors, and history.
"""
class Loc_Agent():
    
    def __init__(self, max_frc, max_vel, rov=None, roc = None, pos_start = None, vel_start = None):
        # if there is no starting position:
        if pos_start is None : 
            # give the Bird a random point
            pos_start = Point.random(constrain2d=True)
        # if there is no starting velocity:
        if vel_start is None : 
            # give the Bird a random initial velocity
            vel_start = Vec.random(constrain2d=True)
            vel_start.normalized()
        
        # set the Bird's position to the initial point
        self.pos = pos_start 
        # set the Bird's velocity to the initial velocity
        self.vel = vel_start 
        # set a limit to the Bird's velocity
        self.max_vel = max_vel
        # set the initial acceleration to zero
        self.acl = Vec() 
        # set a limit to the Bird's acceleration
        self.max_frc = max_frc
        # range of vision is the range to which this bird can see other birds
        if rov == None:
            self.rov = 40.0
        else:
            self.rov = rov
        # range of comfort is the range of comfort with nearby birds
        if roc == None:
            self.roc = 15.00
        else:
            self.roc = roc
        
        # initalize history lists 
        self.clear_history()
        # record this initial condition as a "step"
        self.record()
        
    # define property functions to return x- and y-position
    @property
    def x(self): 
        return self.pos.x
    
    @property
    def y(self): 
        return self.pos.y
        
    """
    Overload the '==' operator so that two birds will be considered equal if their x and y coordinates are equal. Needed in order for the Point.cull_duplicates method to work.
    [noprint]
    """
    def __eq__(self, other): 
        try:
            return all([self.x==other.x,self.y==other.y])
        except:
            return False

    """
    Cause the Bird object to move, using its current velocity and acceleration.
    """        
    def step(self):  
        # add the acceleration to the velocity
        self.vel += self.acl
        # limit to the maximum velocity
        self.vel = self.vel.limited(self.max_vel) 
        # move bird
        self.pos += self.vel 
        # record the step after move
        self.record()       
        # reset acceleration back to zero for next step
        self.acl = Vec() 

    """
    Steer the Bird toward, or away from, a desired vector. Implements Craig Reynolds' formular: Steer = Desired - Velocity
    """  
    def steer(self, desired = Vec(), weight = 1.0, limit = True):
        # if the desired direction is not zero:
        if desired <> Vec():
            # scale the desire vector to the max velocity
            desired = desired.normalized(self.max_vel)
        # Steer = Desired - Velocity
        steer =  desired  - self.vel
        # limit the steer vector and weight it
        if limit:
            steer = steer.limited(self.max_frc) * weight
        else:
            steer = steer * weight
        # add the steer vector to acceleration
        self.acl += steer         

    """
    Basic behavior functions to allow Bird to interact with neighbors
    """   
    # scan a region of space and find birds within it 
    def scan(self, quadtree) :
        # create the boundary to scan
        b_scan = Bounds(center = self.pos, dim_x = self.rov, dim_y = self.rov)
        # get a list of birds within boundary
        close_birds = quadtree.pts_in_bounds(b_scan)
        # return close birds that are not this bird
        return [c for c in close_birds if c is not self]    
        
    # find the birds within a certain distance
    def in_range(self, others, range):
        # get the square of the range distance
        r2 = range**2
        # initialize an empty list
        near_birds = []
        # for each bird:
        for bird in others:
            # get the square of the distance to this bird
            dist2 = self.pos.distance2(bird.pos)
            # if the distance is within the range:
            if dist2 !=0 and dist2 <= r2 : 
                # add it to the list of nearby birds
                near_birds.append(bird)
        return near_birds

    """
    Mid-Level Behaviors to steer the Bird
    """
    # steers this bird towards a group of birds
    def seek(self, others, weight = 1.0):
        # if there are birds in the group:
        if len(others) > 0 : 
            # define goal point as centroid of the birds
            goal = Point.centroid([other_bird.pos for other_bird in others])
            # steer toward the goal point
            self.steer(Vec(self.pos, goal), weight)

    # steers this bird away from a group of birds    
    def flee(self, others, weight = 1.0):
        # if there are birds in the group:
        if len(others) > 0 :
            # start a list of vectors
            vecs = []
            # for every bird in the group:
            for other_bird in others:
                # make a vector from other to self
                vec = Vec(other_bird.pos,self.pos)
                # find square of the length
                dist2 = vec.length2
                # if it is not zero:
                if dist2 > 0 : 
                    # normailze and scale vector
                    vec.normalized(1/dist2)
                # add it to the list
                vecs.append(vec)
            # steer toward the average of the vectors
            self.steer(Vec.average(vecs), weight)           

    # steers this bird to align with other birds
    def align(self,others, weight = 1.0):
        print [other_bird.vel for other_bird in others]
        # if there are birds in the group:
        if len(others) > 0 :     
            # get the average vector of nearby birds
            
            align_vec=Vec.average([other_bird.vel for other_bird in others])
            # steer the average alignment vector
            self.steer(align_vec, weight)
            
            
            
    # steers birds toward or away from attractors
    def attractor(self, attractors = [], repulsors = [], weight = 3.0):
        total = attractors + repulsors
        if len(total) > 0:
            # put together list of vectors
            vecs = []
            for att in total:
                v = Vec(self.pos, Point(att.x, att.y, att.z))
                dist = v.length
                if dist < att.rad:
                    print "att: ", att, v,
                    v = v.normalized((att.rad-dist)/att.rad)
                    print v
                    if att in repulsors:
                        v = v.inverted()
                    vecs.append(v)
            print vecs
            if len(vecs) > 0:
                result_vec = Vec.average(vecs)
                print "attractor: ", result_vec, result_vec.length
                self.steer(result_vec, weight, limit = True)

    """
    Reset the Bird to its initial position and values
    [noprint]
    """    
    def reset(self):
        """
        resets this bird back to his initialized condition.
        replaces existing history lists with single inital values
        """
        self.is_alive = True
        if len(self.hist_pos) > 0:
            self.pos = self.hist_pos[0]
            self.vel = self.hist_vel[0]
            self.acl = self.hist_acl[0]
        self.clear_history()
        self.record()
        
    """
    Functions to clear the history and record a step
    """    
    # clear the position, velocity and acceleration histories
    def clear_history(self):
        self.hist_pos = []
        self.hist_vel = []
        self.hist_acl = []

    # record this step
    def record(self):
        # add its position, velocity and accelaration to the history
        self.hist_pos.append(self.pos)
        self.hist_vel.append(self.vel)
        self.hist_acl.append(self.acl)

