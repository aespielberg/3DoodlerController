##This file is part of pythonOCC.
##
##pythonOCC is free software: you can redistribute it and/or modify
##it under the terms of the GNU Lesser General Public License as published by
##the Free Software Foundation, either version 3 of the License, or
##(at your option) any later version.
##
##pythonOCC is distributed in the hope that it will be useful,
##but WITHOUT ANY WARRANTY; without even the implied warranty of
##MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##GNU Lesser General Public License for more details.
##
##You should have received a copy of the GNU Lesser General Public License
##along with pythonOCC.  If not, see <http://www.gnu.org/licenses/>.

"""

demonstrates overpainting of the OCC OpenGL viewer to share its OpenGL Context
with Qt

this allows for interesting interfaces, where elements are drawn as an overlay

:ref:`Catia's structure browser` is a well known application of this idea

the example is extends Qt's :ref:`OpenGL overpainting example`

.. Catia's structure browser::
        http://goo.gl/KBh7BL

.. OpenGL overpainting example::
        https://github.com/Werkov/PyQt4/blob/master/examples/opengl/overpainting.py


"""


import random
import sys
import IPython
import time
import os
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from Queue import PriorityQueue as PQ



from OCC.Display.qtDisplay import qtViewer3d, get_qt_modules
from OCC.gp import gp_Pnt2d, gp_Pnt
from OCC.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge,
                                BRepBuilderAPI_MakeVertex,
                                BRepBuilderAPI_MakeWire)
from OCC.BRepFill import BRepFill_Filling
from OCC.GeomAbs import GeomAbs_C0
from OCC.GeomAPI import GeomAPI_PointsToBSpline, GeomAPI_ProjectPointOnCurve
from OCC.TColgp import TColgp_Array1OfPnt


QtCore, QtGui, QtOpenGL = get_qt_modules()

SAVE_FILE = "test.txt"
FIXED_FILE = "output.txt"

X_VOL = 0.1
Y_VOL = 0.1
Z_VOL = 0.04


try:
    from OpenGL.GL import (glViewport, glMatrixMode, glOrtho, glLoadIdentity,
                           GL_PROJECTION, GL_MODELVIEW)
                           
except ImportError:
    msg = "for this example, the OpenGL module is required" \
          "why not run \"pip install PyOpenGL\"\?"
    sys.exit(status=1)


class GLWidget(qtViewer3d):
    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)
        #IPython.embed()

        self._initialized = False

        midnight = QtCore.QTime(0, 0, 0)
        random.seed(midnight.secsTo(QtCore.QTime.currentTime()))

        
        self.object = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0
        self.image = QtGui.QImage()
        self.bubbles = []
        self.lastPos = QtCore.QPoint()
        self.lines = []
        self.current_point = None
        self.pts = []
        self.shiftHeld = True
        self.shapesToCurves = {}
        self.workingPoint = None
        self.currentSpline = None
        self.buildGraph = nx.Graph()
        self.currentConnections = {}


        self.trolltechGreen = QtGui.QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)

        self.animationTimer = QtCore.QTimer()
        self.animationTimer.setSingleShot(False)
        self.animationTimer.timeout.connect(self.animate)
        self.animationTimer.start(25)

        self.setAutoFillBackground(False)

        self.setMinimumSize(200, 200)
        self.setWindowTitle("Overpainting a Scene")

        # parameters for overpainting
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground, 0)
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent)
        
    def appendToDict(self, dictionary, key, val):
        if key in dictionary:
            dictionary[key].append(val)
        else:
            dictionary[key] = [val]  
        
    def delete(self, shape):
        rerender = {}
        for shape1 in self.shapesToCurves:
            if shape1.IsEqual(shape):
                no_rerender = self.shapesToCurves[shape]
        for shape1 in self.shapesToCurves:
            if self.shapesToCurves[shape1] != no_rerender:
                rerender[shape1] = self.shapesToCurves[shape1]
        self._display.EraseAll()
        print self.shapesToCurves
        for shape1 in rerender:
            self._display.DisplayShape(shape1)
            
        self.shapesToCurves = rerender
        self._display.Viewer.Grid().GetObject().Display()
        
    def lookupSpline(self, spline):
        for shape in self.shapesToCurves:
            if shape.IsEqual(spline):
                return self.shapesToCurves[shape]
        return None #didn't find anything
        
    def findCurvesRoot(self):
        return [curve for curve in self.buildGraph if self.isCurveRoot(curve.GetObject())]
        
    def isCurveRoot(self, curve, res=10000):
        last = curve.LastParameter()
        first = curve.FirstParameter()
        prevVal = None
        for i in range(res):
            u = first + (last - first)/res * i
            point = curve.Value(u)
            if prevVal is not None:
                if np.sign(prevVal.Z()) * np.sign(point.Z()) <= 0: #if it intersects the z axis
                    return True
            prevVal = point
        return False
        
    def minDistFromCenter(self, curve, mid_x, mid_y, res=10000):
        last = curve.GetObject().LastParameter()
        first = curve.GetObject().FirstParameter()
        min_dist = sys.float_info.max
        prevVal = None
        for i in range(res):
            u = first + (last - first)/res * i
            point = curve.GetObject().Value(u)
            x_diff = point.X() - mid_x
            y_diff = point.Y() - mid_y
            dist = np.sqrt(x_diff * x_diff + y_diff * y_diff)
            if  dist < min_dist:
                min_dist = dist
        return min_dist
        
    def prune_lists(self, all_samples):
        tol = 0.01
        new_all_samples = []
        for ii in range(len(all_samples)):
            prevPoint = gp_Pnt(sys.float_info.max, sys.float_info.max, sys.float_info.max)
            new_list = []
            for jj in range(len(all_samples[ii])):
                point = all_samples[ii][jj]
                if point.Distance(prevPoint) > 0.01: #TODO: should probably be concerned about pathological, dense whirly curves
                    new_list.append(point)
                    prevPoint = point #otherwise keep it and continue
            new_all_samples.append(new_list)
        return new_all_samples
        
    def sampleCurve(self, curve, res=10):
        tol = 0.0015
        uThresh = curve.Resolution(tol)
        sample = []
        last = curve.LastParameter()
        first = curve.FirstParameter()
        
        
        res = (last - first) / uThresh
        for i in range(int(res)):
            u = first + (last - first)/res * i
            point = curve.Value(u)
            sample.append(point)
            
            
        #Finally, add the last point:
        sample.append(curve.Value(last))
        return sample
        
    def getCenter(self, res=10000):
        max_x = -sys.float_info.max
        max_y = -sys.float_info.max
        min_x = sys.float_info.max
        min_y = sys.float_info.max
        
        for curve in self.buildGraph:
            
        
        
            last = curve.GetObject().LastParameter()
            first = curve.GetObject().FirstParameter()
            #First get bounding box
            
            
            for i in range(res):
                u = first + (last - first)/res * i
                point = curve.GetObject().Value(u)
                if point.X() > max_x:
                    max_x = point.X()
                if point.Y() > max_y:
                    max_y = point.Y()
                if point.X() < min_x:
                    min_x = point.X()
                if point.Y() < min_y:
                    min_y = point.Y()
        
        x_middle = (max_x + min_x) / 2
        y_middle = (max_y + min_y) / 2
        
        return (x_middle, y_middle)
        
    def fit_to_volume(self, all_samples):
        #TODO: use PCA to rotate?
        
        #First get bounding box
        max_x = -sys.float_info.max
        max_y = -sys.float_info.max
        max_z = -sys.float_info.max
        min_x = sys.float_info.max
        min_y = sys.float_info.max
        min_z = sys.float_info.max
        
        
        
        for sample in all_samples:
            for point in sample:
                if point.X() > max_x:
                    max_x = point.X()
                if point.Y() > max_y:
                    max_y = point.Y()
                if point.Z() > max_z:
                    max_z = point.Z()
                if point.X() < min_x:
                    min_x = point.X()
                if point.Y() < min_y:
                    min_y = point.Y()
                if point.Z() < min_z:
                    min_z = point.Z()
                    
        
                    
        x_range = max_x - min_x
        y_range = max_y - min_y
        z_range = max_z - min_z
        
        x_middle = (max_x + min_x) / 2
        y_middle = (max_y + min_y) / 2
                
        if x_range > 0:
            scale_x = X_VOL / x_range
        else:
            scale_x = 1.
        if y_range > 0:
            scale_y = Y_VOL / y_range
        else:
            scale_y = 1.
        if z_range > 0:
            scale_z = Z_VOL / z_range
        else:
            scale_z = 1.
        
        
        print X_VOL
        print Y_VOL
        print Z_VOL
        
        print x_range
        print y_range
        print z_range
        
        print scale_x
        print scale_y
        print scale_z
                
        scale = np.min([scale_x, scale_y, scale_z]) #Get the biggest downscale
                
        #Now that we have how much we should scale everything, we should center in x and y, scale everything, and return it.
        scaled_samples = []
        for sample in all_samples:
            scaled_sample = []
            
            for point in sample:
                scaled_point = gp_Pnt((point.X() - x_middle) * scale, (point.Y() - y_middle) * scale, point.Z() * scale)
                scaled_sample.append(scaled_point)
                
            scaled_samples.append(scaled_sample)
                
        return scaled_samples            
                    
    def graph_to_ordering(self, graph, roots, mid_x, mid_y):
        pq = PQ()
        curves = []
        print 'roots in gto'
        print roots
        for node in roots:
            print 'min dist'
            print self.minDistFromCenter(node, mid_x, mid_y)
            pq.put(node, self.minDistFromCenter(node, mid_x, mid_y))

                
        while not pq.empty(): #While there are still edges
            print 'in da loop again'
            node = pq.get()
            curves.append(node)
            edges = graph.edges(node, data=True)
            for edge in edges:
                if not (edge[1] in curves): #if sink of edge hasn't been seen yet, no point in adding it twice
                    pq.put( edge[1] , self.minDistFromCenter(edge[1], mid_x, mid_y) )
                
        return curves
        
    
    def add_node_to_queue_z(self, node, graph, pq):
        edges = graph.edges(node, data=True)
        for edge in edges:
            min_con_val = sys.float_info.max
            for conn_point in edge[2]['points']:
                print conn_point
                if conn_point.Z() < min_con_val:
                    min_con_val = conn_point.Z()
            pq.put( (node, edge), min_con_val )
                
        return pq
    
    def fix_graph(self, roots):
        graph = self.buildGraph
        fixed_graph = nx.Graph()
        pq = PQ()
        
        for node in roots:
            self.add_node_to_queue_z(node, graph, pq)
            fixed_graph.add_node(node)
            
                        
        #Now the priority queue has been initialized
        while not pq.empty(): #While there are still edges
            print fixed_graph.number_of_nodes()
            (node, edge) = pq.get()
            print 'in the loop'
            if not (edge[1] in fixed_graph): #if sink of edge hasn't been added yet
                fixed_graph.add_node(edge[1])
                fixed_graph.add_edge(node, edge[1], points=edge[2])
                self.add_node_to_queue_z(edge[1], graph, pq)
            
        return fixed_graph
        
    def sampleToArm(self):
    
        #TODO: First, order the splines through a graph traversal.  keep greedily adding the lowest connection point, guarantees never make "dangles."
        #Use this to prevent cycles.
        #Remove edges contributing to cycles.
        
        roots = self.findCurvesRoot()
        
        print 'roots'
        print roots
        print ''

        cycleless_graph = self.fix_graph(roots)
        
        
        nx.draw(cycleless_graph)
        plt.savefig("graph.pdf")
        
        plt.clf()
        
        nx.draw(self.buildGraph)
        plt.savefig("graph2.pdf")
        
        
        
        
        #Second, use heuristic of inward before outward.
        (mid_x, mid_y) = self.getCenter()
        curves = self.graph_to_ordering(cycleless_graph, roots, mid_x, mid_y)
        

        #Third, sample
        all_samples = self.saveCurvesToFile(curves)
        
        #Fourth, convert to a reasonable print volume
        converted_samples = self.fit_to_volume(all_samples)
        
        #Finally, prune that list
        pruned_samples = self.prune_lists(converted_samples)
        
        #TODO: refactor to share code with all the existing save files
        try:
            os.remove(FIXED_FILE)
        except:
            print 'nothing to remove again!'
            
        with open(FIXED_FILE, "a") as myfile:
            for sample in pruned_samples:
                for point in sample:
                    myfile.write(str(point.X()) + " " + str(point.Y()) + " " + str(point.Z()) + "\n")   
                myfile.write("-\n")    
        
        print 'wrote to file!'
        

        
        
        
            
        
    def saveSamplesToFile(self, sample):
        try:
            os.remove(SAVE_FILE)
        except:
            print 'nothing to remove!'
            
        with open(SAVE_FILE, "a") as myfile:
            for point in sample:
                myfile.write(str(point.X()) + " " + str(point.Y()) + " " + str(point.Z()) + "\n")    

    def saveCurvesToFile(self, curves):
        #curves = list(set(self.shapesToCurves.values()))
        all_samples = []
        for curve in curves:
            sample = self.sampleCurve(curve.GetObject())
            self.saveSamplesToFile(sample)
            with open(SAVE_FILE, "a") as myfile:
                myfile.write("-\n") #delimiter for ending a spline
            all_samples.append(sample)
            
        return all_samples
            
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Delete:
            currentSpline = self._display.GetSelectedShape()
            if currentSpline is not None and currentSpline is not False:
                self.delete(currentSpline)
                
        if event.key() == QtCore.Qt.Key_S and (event.modifiers() & QtCore.Qt.ControlModifier):
            print 'saving time!'
            #all_samples = self.saveCurvesToFile()
            self.sampleToArm() 
                
                
    def mouseReleaseEvent(self, event):
        
        super(GLWidget, self).mouseReleaseEvent(event)
        self.currentSpline =  self._display.GetSelectedShape()
            
    

    def mousePressEvent(self, event):

        self.lastPos = event.pos()
        
        super(GLWidget, self).mousePressEvent(event)
        

        
        """
        worldCoords = super(GLWidget, self).mapToGlobal( self.lastPos )
        print self.lastPos
        """
        
        print self.currentSpline
        corCurve = None #curve corresponding to the selected spline

        if self.currentSpline is not None and self.currentSpline is not False:
            corCurve = self.lookupSpline(self.currentSpline)
            #self.currentSpline = self.shapesToCurves[self.currentSpline] #take the shape and get the real curve from it
        
        if event.buttons() & QtCore.Qt.LeftButton:
            #self.currentSpline = self._display.GetSelectedShape()
            #print self.currentSpline
            pass
            
        
        elif event.buttons() & QtCore.Qt.RightButton and not (event.modifiers() & QtCore.Qt.ShiftModifier):
            print 'first'
            
            
            (x, y, z, vx, vy, vz) = self._display.View.ConvertWithProj(self.lastPos.x(), self.lastPos.y())
            

            
            
            if not self.shapesToCurves:
                #z' = vz*t + z, solve for where z' = 0
                t = -z/vz
                x = x + vx*t
                y = y + vy*t
                z = 0
                
            point = gp_Pnt(x, y, z)
            
            
            if corCurve is not None and corCurve is not False: #something is selected
                print 'not none'
                #get project onto it
                projection = GeomAPI_ProjectPointOnCurve(point, corCurve)
                point = projection.NearestPoint()

                self.workingPoint = point #TODO: should I just make this none-able?
                self.appendToDict(self.currentConnections, corCurve, point)#Add a connection
              
            elif self.workingPoint is not None:
                view_dir = self._display.View.ViewOrientation().ViewReferencePlane().Coord() #Note that this is backwards!
                d = -view_dir[0]*(self.workingPoint.X() - x) -view_dir[1]*(self.workingPoint.Y() - y) -view_dir[2]*(self.workingPoint.Z() - z)
                d /=  (-view_dir[0]*vx) + (-view_dir[1]*vy) + (-view_dir[2]*vz)
                x = x + vx*d
                y = y + vy*d
                z = z + vz*d
                #TODO: WIP
                point = gp_Pnt(x, y, z)
            
            
            self._display.DisplayShape(point, update=False)

                
            self.pts.append(point)
            
        elif event.buttons() & QtCore.Qt.RightButton and (event.modifiers() & QtCore.Qt.ShiftModifier):
            print 'second'
            print self.pts
            curve = self.points_to_bspline(self.pts)
            
            #Add the curve to the network:
            self.buildGraph.add_node(curve)
            for s in self.currentConnections:
                self.buildGraph.add_edge(s, curve, points = self.currentConnections[s]) #from that spline to this current curve
            
            self._display.DisplayShape(curve, update=False)

            self._display.View.SetZoom(1.0) #force a repaint

            self.pts = [] #clear it
            #self._display.View.SetZSize(1.0)
            
            self._display.SelectArea(0, 0, 2000, 2000) #select everything and iterate through it

            
            shapes = self._display.GetSelectedShapes()
            
            for shape in shapes:
                known = False
                for knownShape in self.shapesToCurves:
                    if shape.IsEqual(knownShape):
                        known = True
                        break
                if known:
                    continue
                #Did not see it
                self.shapesToCurves[shape] = curve

            print self.shapesToCurves
            
            self.workingPoint = None #reset this value
            self.currentConnections = {} #reset this value
            
            
        

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        """
        if (event.buttons() & QtCore.Qt.LeftButton):
            self.setXRotation(self.xRot + 8 * dy)
            self.setYRotation(self.yRot + 8 * dx)
        
        
        
        elif (event.buttons() & QtCore.Qt.RightButton):
            self.setXRotation(self.xRot + 8 * dy)
            self.setZRotation(self.zRot + 8 * dx)
        """
        

        self.lastPos = event.pos()
        if not event.buttons() & QtCore.Qt.RightButton:
            super(GLWidget, self).mouseMoveEvent(event)

    def paintGL(self):
        if self._inited:
            self._display.Context.UpdateCurrentViewer()

    def paintEvent(self, event):
        if self._inited:

            self._display.Context.UpdateCurrentViewer()
            self.makeCurrent()
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)

            if self.context().isValid():
                self.swapBuffers()

                
                if self._drawbox:
                    painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 1))
                    rect = QtCore.QRect(*self._drawbox)
                    painter.drawRect(rect)

                """
                for bubble in self.bubbles:
                    if bubble.rect().intersects(QtCore.QRectF(event.rect())):
                        bubble.drawBubble(painter)
                """

                painter.end()
                self.doneCurrent()
            else:
                print('invalid OpenGL context: Qt cannot overpaint viewer')

    def showEvent(self, event):
        pass
        #self.createBubbles(20 - len(self.bubbles))

    def sizeHint(self):
        return QtCore.QSize(400, 400)


    def animate(self):
        pass
        """
        for bubble in self.bubbles:
            bubble.move(self.rect())
        self.update()
        """

    def setupViewport(self, width, height):
        side = min(width, height)
        glViewport((width - side) // 2, (height - side) // 2, side, side)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0)
        glMatrixMode(GL_MODELVIEW)
        
    def points_to_bspline(self, pnts):
        pts = TColgp_Array1OfPnt(0, len(pnts)-1)
        for n, i in enumerate(pnts):
            pts.SetValue(n, i)
        crv = GeomAPI_PointsToBSpline(pts)
        return crv.Curve()




if __name__ == '__main__':
    def TestOverPainting():
        class AppFrame(QtGui.QWidget):
            def __init__(self, parent=None):
                QtGui.QWidget.__init__(self, parent)
                self.setWindowTitle(self.tr("qtDisplay3d overpainting example"))
                self.resize(640, 480)
                self.canva = GLWidget(self)
                mainLayout = QtGui.QHBoxLayout()
                mainLayout.addWidget(self.canva)
                mainLayout.setContentsMargins(0, 0, 0, 0)
                self.setLayout(mainLayout)
                
                

            def runTests(self):
                self.canva._display.Test()

        app = QtGui.QApplication(sys.argv)
        frame = AppFrame()
        frame.show()
        frame.canva.InitDriver()
        frame.canva._display.Viewer.Grid().GetObject().Display() #turn on the grid
        #frame.runTests()
        app.exec_()

    TestOverPainting()
