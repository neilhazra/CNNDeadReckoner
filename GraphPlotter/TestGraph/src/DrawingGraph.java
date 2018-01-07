import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

public class DrawingGraph
{
    public static void main ( String[] args )
    {
        JFrame frame = new JFrame ("Drawing a Graph");
        DrawingPanel dPanel = new DrawingPanel();
        frame.getContentPane().add(dPanel);
        frame.setSize (800, 600);
        frame.setLocation (40, 40);
        frame.setVisible (true);
        frame.getContentPane().repaint();
        frame.setDefaultCloseOperation (JFrame.EXIT_ON_CLOSE);
    }
}