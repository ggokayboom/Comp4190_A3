__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
import time
from pathplanning import PathPlanningProblem, Rectangle
from astar import AStar

class CellDecomposition:
    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = [Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', []]

    def Draw(self, ax, node = None):
            if ( node == None ):
                node = self.root
            r = plt.Rectangle((node[0].x, node[0].y), node[0].width, node[0].height, fill=False, facecolor=None, alpha=0.5)
            if ( node[1] == 'mixed' ):
                color = '#5080ff'
                if ( node[2] == [] ):
                    r.set_fill(True)
                    r.set_facecolor(color)
            elif ( node[1] == 'free' ):
                color = '#ffff00'
                r.set_fill(True)
                r.set_facecolor(color)
            elif ( node[1] == 'obstacle'):
                color = '#5050ff'
                r.set_fill(True)
                r.set_facecolor(color)
            else:
                print("Error: don't know how to draw cell of type", node[1])
            #print('Draw node', node)
            ax.add_patch(r)
            for c in node[2]:
                self.Draw(ax, c)

    def CountCells(self, node = None ):
        if ( node is None ):
            node = self.root
        sum = 0
        if ( node[2] != [] ):
            sum = 0
            for c in node[2]:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum

class QuadTreeDecomposition(CellDecomposition):
    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        self.free_nodes = []
        self.root = self.Decompose(self.root)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break
        
        if ( cell == 'mixed'):
            if (rwidth / 2.0 > self.minimumSize) and (rheight / 2.0 > self.minimumSize):
                childt1 = [Rectangle(rx, ry, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild1 = self.Decompose( childt1 )
                childt2 = [Rectangle(rx + rwidth/2.0, ry, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild2 = self.Decompose( childt2 )
                childt3 = [Rectangle(rx, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild3 = self.Decompose( childt3 )
                childt4 = [Rectangle(rx + rwidth/2.0, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild4 = self.Decompose( childt4 )
                children = [ qchild1, qchild2, qchild3, qchild4 ]
                node[2] = children
            else:
                cell = 'obstacle'
        node[1] = cell
        if ( node[1] == 'free'):
            self.free_nodes.append(node)
        return node

class BinarySpacePartitioning(CellDecomposition):
    def __init__(self, domain, minimumSize ):
        super().__init__(domain, minimumSize)
        self.free_nodes = []
        self.root = self.Decompose(self.root)

    def Entropy(self, p):
        e = 0.0
        if ( ( p > 0 ) and ( p < 1.0 ) ):
            e = -p * math.log(p,2) - (1-p) * math.log(1-p,2)
        return e

    def CalcEntropy(self, rect):
        area = rect.width * rect.height
        a = 0.0
        for o in self.domain.obstacles:
            a = a + rect.CalculateOverlap(o)
        p = a / area
        return self.Entropy(p)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height
        area = rwidth * rheight

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break

        if ( cell == 'mixed'):
            entropy = self.CalcEntropy(r)
            igH = 0.0
            hSplitTop = None
            hSplitBottom = None
            vSplitLeft = None
            vSplitRight = None
            if ( r.height / 2.0 > self.minimumSize):
                hSplitTop = Rectangle(rx, ry + rheight/2.0, rwidth, rheight/2.0)
                entHSplitTop = self.CalcEntropy(hSplitTop)
                hSplitBottom = Rectangle(rx, ry, rwidth, rheight/2.0)
                entHSplitBottom = self.CalcEntropy( hSplitBottom )

                igH = entropy - ( r.width * r.height / 2.0 ) / area * entHSplitTop \
                      - ( r.width * r.height / 2.0 ) / area * entHSplitBottom
            igV = 0.0
            if ( r.width / 2.0 > self.minimumSize ):
                vSplitLeft = Rectangle(rx, ry, rwidth/2.0, rheight )
                entVSplitLeft = self.CalcEntropy( vSplitLeft )
                vSplitRight = Rectangle( rx + rwidth/2.0, ry, rwidth/2.0, rheight)
                entVSplitRight = self.CalcEntropy( vSplitRight)
                igV = entropy - ( r.width/2.0 * r.height ) / area * entVSplitLeft \
                      - ( r.width/2.0 * r.height ) / area * entVSplitRight
            children = []
            if ( igH > igV ):
                if ( igH > 0.0 ):
                    if ( hSplitTop is not None ) and ( hSplitBottom is not None ):
                        childTop = [ hSplitTop, 'unknown', [] ]
                        childBottom = [hSplitBottom, 'unknown', [] ]
                        children = [ childTop, childBottom]
            else:
                if ( igV > 0.0 ):
                    if ( vSplitLeft is not None ) and ( vSplitRight is not None ):
                        childLeft = [vSplitLeft, 'unknown', [] ]
                        childRight = [ vSplitRight, 'unknown', [] ]
                        children = [ childLeft, childRight ]
            for c in children:
                self.Decompose(c)
            node[2] = children
        node[1] = cell
        if ( node[1] == 'free'):
            self.free_nodes.append(node)
        return node

def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 10.0
    height = 10.0

    obs_count = int(argv[0])
    # obs_count = 40
    iterations = 1 # specify # of times to run map generation & Quadtree, FBSP, RRT (used for comparison of run times)
    # avg times
    # TODO: add RRT averages
    qtd_decomp_avg = 0.0
    qtd_star_avg = 0.0
    qtd_path_avg = 0.0
    fbsp_decomp_avg = 0.0
    fbsp_star_avg = 0.0
    fbsp_path_avg = 0.0

    if obs_count >= 30:
        obs_width = obs_height = 2.0
    else:
        obs_width = obs_height = 5.0

    for i in range(iterations):
        pp = PathPlanningProblem( width, height, obs_count, obs_width, obs_height)

        qtd_start = time.perf_counter()
        qtd = QuadTreeDecomposition(pp, 0.1)
        qtd_end = time.perf_counter()
        initial, goals = pp.CreateProblemInstance()

        fig = plt.figure()
        ax = fig.add_subplot(1,2,1, aspect='equal')
        ax.set_xlim(0.0, width)
        ax.set_ylim(0.0, height)

        for o in pp.obstacles:
            ax.add_patch(copy.copy(o.patch) )
        ip = plt.Rectangle((initial.x,initial.y), initial.width, initial.height, facecolor='#ff0000')
        ax.add_patch(ip)

        # for g in goals:
        g = plt.Rectangle((goals.x,goals.y), goals.width, goals.height, facecolor='#00ff00')
        ax.add_patch(g)
        # print(initial.x, initial.y)
        # print(goals.x, goals.y)
        # qtd.Draw(ax)
        # plt.show()

        # run A*
        qtd_star_start_time = time.perf_counter()
        qtd_path = AStar(qtd,initial,goals).path_to_goal
        qtd_star_end_time = time.perf_counter()
        qtd_path_len = 0
        x_cord = []
        y_cord = []
        for cord in qtd_path:
            x_cord += [cord[0]]
            y_cord += [cord[1]]
            qtd_path_len += cord[2]
        plt.plot(x_cord,y_cord,'-')

        qtd.Draw(ax)
        n = qtd.CountCells()
        ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

        ax = fig.add_subplot(1,2,2, aspect='equal')
        ax.set_xlim(0.0, width)
        ax.set_ylim(0.0, height)

        fbsp_start = time.perf_counter()
        bsp = BinarySpacePartitioning(pp, 0.1)
        fbsp_end = time.perf_counter()

        for o in pp.obstacles:
            ax.add_patch(copy.copy(o.patch))
        ip = plt.Rectangle((initial.x,initial.y), initial.width, initial.height, facecolor='#ff0000')
        ax.add_patch(ip)

        # for g in goals:
        g = plt.Rectangle((goals.x,goals.y), goals.width, goals.height, facecolor='#00ff00')
        ax.add_patch(g)

        # run A*
        fbsp_star_start_time = time.perf_counter()
        fbsp_path = AStar(bsp,initial,goals).path_to_goal
        fbsp_star_end_time = time.perf_counter()
        
        fbsp_path_len = 0

        x_cord = []
        y_cord = []
        for cord in fbsp_path:
            x_cord += [cord[0]]
            y_cord += [cord[1]]
            fbsp_path_len += cord[2]
        plt.plot(x_cord,y_cord,'-')

        bsp.Draw(ax)
        n = bsp.CountCells()
        ax.set_title('BSP Decomposition\n{0} cells'.format(n))

        plt.show()

        qtd_decomp_time = qtd_end - qtd_start
        qtd_star_time = qtd_star_end_time - qtd_star_start_time
        qtd_decomp_avg += qtd_decomp_time
        qtd_star_avg += qtd_star_time
        qtd_path_avg += qtd_path_len
        fbsp_decomp_time = fbsp_end - fbsp_start
        fbsp_star_time = fbsp_star_end_time - fbsp_star_start_time
        fbsp_decomp_avg += fbsp_decomp_time
        fbsp_star_avg += fbsp_star_time
        fbsp_path_avg += fbsp_path_len
        
        print('\n')
        if len(qtd_path) < 1:
            print('No path found for Quadtree A*')
        
        print("Quadtree Decom Runtime: ", qtd_decomp_time)
        print('Quadtree A* Runtime: ', qtd_star_time)
        print('Quadtree A* path length: ', qtd_path_len)

        if len(fbsp_path) < 1:
            print('No path found for FBSP A*')

        print("FBSP Decomp Runtime: ", fbsp_decomp_time)
        print('FBSP A* Runtime: ', fbsp_star_time)
        print('FBSP A* path length: ', fbsp_path_len)
        print('\n')

    if iterations > 1:
        print("Quadtree AVG Decom Runtime: ", qtd_decomp_avg/iterations)
        print('Quadtree AVG A* Runtime: ', qtd_star_avg/iterations)
        print('Quadtree AVG A* path length: ', qtd_path_avg/iterations)
        print("FBSP AVG Decomp Runtime: ", fbsp_decomp_avg/iterations)
        print('FBSP AVG A* Runtime: ', fbsp_star_avg/iterations)
        print('FBSP AVG A* path length: ', fbsp_path_avg/iterations)
        
if ( __name__ == '__main__' ):
    main()


