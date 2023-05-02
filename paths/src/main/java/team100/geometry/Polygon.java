package team100.geometry;

public class Polygon extends Shape {
    public Polygon(Point... points) {
        for (Point p : points) {
            addPoint(p);
        }
    }
}
