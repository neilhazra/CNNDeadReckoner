import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

class DrawingPanel extends JPanel
{
    public void paintComponent ( Graphics g )
    {
        super.paintComponent ( g );
        // Set up a Cartesian coordinate system

        // get the size of the drawing area
        Dimension size = this.getSize();

        // place the origin at the middle
        g.translate (size.width / 2, size.height / 2);

        // draw the x and y axes
        drawXYAxes (g);

        graphLine (g, 0.5, 25.2);
        graphQuadratic (g, -0.125, 0, 0);

    }

    private void drawXYAxes (Graphics g) {
        Dimension size = this.getSize();
        int hBound = size.width / 2;
        int vBound = size.height / 2;
        int tic = size.width / 100;

        // draw the x-axis
        g.drawLine (-hBound, 0, hBound, 0);

        // draw the tic marks along the x axis
        for (int k = -hBound; k <= hBound; k += 10)
            g.drawLine (k, tic, k, -tic);

        // draw the y-axis
        g.drawLine (0, vBound, 0, -vBound);

        // draw the tic marks along the y axis
        for (int k = -vBound; k <= vBound; k += 10)
            g.drawLine (-tic , k, +tic, k);
    }

    private void graphLine (Graphics g, double a,  double b) {
        Dimension size = this.getSize();
        g.setColor (Color.red);

        int x1 = size.width / 2;
        int y1 = (int) ((a * x1) + b);
        y1 = - y1;

        int x2 = - x1;
        int y2 = (int)((a * x2) + b);
        y2 = - y2;

        g.drawLine (x1, y1, x2, y2);
    }

    private void graphQuadratic (Graphics g, double a, double b, double c) {
        Dimension size = this.getSize();
        g.setColor (Color.blue);

        int hBound = size.width / 2;
        for (int i = -hBound; i <= hBound; i++) {
            int x1 = i;
            int y1 = (int) (a * x1 * x1 + b * x1 + c);
            y1 = - y1;

            int x2 = x1 + 1;
            int y2 = (int) (a * x2 * x2 + b * x2 + c);
            y2 = - y2;

            g.drawLine (x1, y1, x2, y2);
        }

    }
}