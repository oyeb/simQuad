#!/usr/bin/env python
import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as P
import matplotlib.mlab as M
import matplotlib
import numpy as N
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg as FigureCanvas
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg as NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
from matplotlib.widgets import Button as mButton
from matplotlib.widgets import CheckButtons

import os, sys, Tkinter
import tkFileDialog
import Tkconstants as Tkc

t = N.linspace(0, 10, 1e3)
x = N.cos(2*N.pi*t)
y = N.sin(2*N.pi*t)

class window(Tkinter.Tk):
    def __init__(self, parent):
        Tkinter.Tk.__init__(self, parent)
        self.protocol('WM_DELETE_WINDOW', self._kill)
        self._create_widgets()
       
    def _kill(self):
        self.destroy()
        sys.exit()

    def _create_widgets(self):
        self._create_canvas()
        self._create_toolbar()
        self._create_menu()
        return

    def _create_toolbar(self):
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.update()
        self.canvas._tkcanvas.pack()
        #self.toolbar.pack()
        return

    def _create_canvas(self):
        self.fig = P.figure()
        self.fig = P.figure(figsize=(7,7))
        self.axes = []
        self.axes.append( self.fig.add_subplot(111) )
        self.axes.append( self.fig.add_subplot(211) )
        self.axes.append( self.fig.add_subplot(212) )
        
        self.axes[0].set_visible(True)
        self.axes[1].set_visible(False)
        self.axes[2].set_visible(False)

        self.lines = []
        self.lines.append( self.axes[0].plot([0.],[0.], '.')[0] )
        self.lines.append( self.axes[1].plot([0.],[0.], '.')[0] )
        self.lines.append( self.axes[2].plot([0.],[0.], '.')[0] )

        self.fig.subplots_adjust(bottom=0.175)

        self.frame = Tkinter.Frame(self)
        self.frame.pack(padx=0, pady=0)
        self.canvas = FigureCanvas(self.fig, master=self.frame)
        self.canvas.show()
        self.canvas.get_tk_widget().pack(side='top', fill='both')
        #self.canvas._tkcanvas.pack(side='top', fill='both', expand=1)
        return

    def _plot_it(self):
        print "hello"
        print "which_one: ", self.menu_val.get()
        if self.menu_val.get() == self.items[0]:
            print "in one"
            self.axes[0].set_visible(True)
            self.axes[1].set_visible(False)
            self.axes[2].set_visible(False)
            self.lines[0].set_xdata(x)
            self.lines[0].set_ydata(y)
            self.axes[0].set_xlabel('x')
            self.axes[0].set_ylabel('y')
            self.axes[0].set_xlim([min(x), max(x)])
            self.axes[0].set_ylim([min(y), max(y)])
            print "x: ", N.min(x), N.max(x)
            print "y: ", N.min(y), N.max(y)
            P.draw()
        else:
            self.axes[0].set_visible(False)
            self.axes[1].set_visible(True)
            self.axes[2].set_visible(True)
            self.lines[1].set_xdata(t)
            self.lines[1].set_ydata(x)
            self.lines[2].set_xdata(t)
            self.lines[2].set_ydata(x)
            
            self.axes[1].set_ylabel('x(t)')
            self.axes[1].set_xlim([N.min(t), N.max(t)])
            self.axes[1].set_ylim([N.min(x), N.max(x)])
            
            self.axes[2].set_ylabel('y(t)')
            self.axes[2].set_xlim([N.min(t), N.max(t)])
            self.axes[2].set_ylim([N.min(y), N.max(y)])
            self.axes[2].set_xlabel('t')
            P.draw()

    def _create_menu(self):
        self.items = ['scatter', 'strip']
        self.menu_val = Tkinter.StringVar()
        self.menu_val.set(self.items[0])
        
        self.button_menu_frame = Tkinter.Frame(self)
        def click(): print "click!"
        self.button = Tkinter.Button(self.button_menu_frame, text='plot', command=self._plot_it)
        self.button.pack(side='top', ipadx=63, fill='both')

        self.menu_frame = Tkinter.Frame(self.button_menu_frame)
        #self.menu_frame.pack(side='left', ipadx=20, expand=1, fill='both')
        self.menu = Tkinter.OptionMenu(self.menu_frame, self.menu_val, *self.items)
        self.menu.pack(side='left', anchor=Tkc.SW, ipadx=20, expand=1, fill='both')

        self.menu_frame.pack(side='top', fill='both')
        self.button_menu_frame.pack(side='left', fill='both')
        return

if __name__ == "__main__":
    app = window(None)
    app.title('Analysis')
    #app.pack_propegate(0)
    app.mainloop()
