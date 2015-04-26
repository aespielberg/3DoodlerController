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
        self.workingWithProjection = False


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
                

    
            
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Delete:
            currentSpline = self._display.GetSelectedShape()
            if currentSpline is not None:
                self.delete(currentSpline)

    def mousePressEvent(self, event):
        #IPython.embed()
        self.lastPos = event.pos()
        
        super(GLWidget, self).mousePressEvent(event)
        

        
        """
        worldCoords = super(GLWidget, self).mapToGlobal( self.lastPos )
        print self.lastPos
        """
        currentSpline =  self._display.GetSelectedShape()
        print currentSpline
        if currentSpline is not None:
            currentSpline = self.shapesToCurves[currentSpline] #take the shape and get the real curve from it
        
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
            
            if currentSpline is not None: #something is selected
                print 'not none'
                #get project onto it
                projection = GeomAPI_ProjectPointOnCurve(point, currentSpline)
                point = projection.NearestPoint()
                self.workingWithProjection = True #Future things should work in this point's plane intersection
                self.workingPoint = point #TODO: should I just make this none-able?
                
            elif self.workingWithProjection:
                view_dir = self._display.View.ViewOrientation().ViewReferencePlane().Coord() #Note that this is backwards!
                t = -z/vz
                x = x + vx*t
                y = y + vy*t
                z = 0
                #TODO: WIP
                #point = gp_Pnt(x, y, z)
            
            self._display.DisplayShape(point, update=False)

                
            self.pts.append(point)
            
        elif event.buttons() & QtCore.Qt.RightButton and (event.modifiers() & QtCore.Qt.ShiftModifier):
            print 'second'
            curve = self.points_to_bspline(self.pts)
            self._display.DisplayShape(curve, update=False)

            self._display.View.SetZoom(1.0) #force a repaint

            self.pts = [] #clear it
            #self._display.View.SetZSize(1.0)
            
            self._display.SelectArea(0, 0, 2000, 2000) #select everything and iterate through it

            
            shapes = self._display.GetSelectedShapes()
            
            for shape in shapes:
                for knownShape in self.shapesToCurves:
                    if shape.IsEqual(knownShape):
                        continue
                #Did not see it
                self.shapesToCurves[shape] = curve
            print self.shapesToCurves
            
            self.workingWithProjection = False #reset this value
            
            
        

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
