package team100.geometry;

import java.util.ArrayList;

public abstract class Shape {

    private final ArrayList<Point> points = new ArrayList<Point>();

    protected void addPoint(Point p) {
        if (getPoints().contains(p)) { // don't add it again
            return;
        }
        points.add(p);
    }

    public ArrayList<Point> getPoints() {
        return points;
    }

    public Point getPoint(int index) {
        return getPoints().get(index);
    }

    public int getPointCount() {
        return points.size();
    }

    public boolean intersects(Shape shape) {
        /*
         * Intersection formula used:
         * (x4 - x3)(y1 - y3) - (y4 - y3)(x1 - x3)
         * UA = ---------------------------------------
         * (y4 - y3)(x2 - x1) - (x4 - x3)(y2 - y1)
         * 
         * (x2 - x1)(y1 - y3) - (y2 - y1)(x1 - x3)
         * UB = ---------------------------------------
         * (y4 - y3)(x2 - x1) - (x4 - x3)(y2 - y1)
         * 
         * if UA and UB are both between 0 and 1 then the lines intersect.
         * 
         * Source: http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
         */

        for (int i = 0; i < getPointCount(); ++i) {
            int iNext = i + 1;
            if (iNext >= getPointCount()) {
                iNext = 0;
            }

            for (int j = 0; j < shape.getPointCount(); ++j) {
                int jNext = j + 1;
                if (jNext >= shape.getPointCount()) {
                    jNext = 0;
                }

                double x1 = shape.getPoint(j).getX();
                double y1 = shape.getPoint(j).getY();
                double x2 = shape.getPoint(jNext).getX();
                double y2 = shape.getPoint(jNext).getY();
                double x3 = getPoint(i).getX();
                double y3 = getPoint(i).getY();
                double x4 = getPoint(iNext).getX();
                double y4 = getPoint(iNext).getY();

                double unknownA = (((x4 - x3) * (y1 - y3)) - ((y4 - y3) * (x1 - x3))) /
                        (((y4 - y3) * (x2 - x1)) - ((x4 - x3) * (y2 - y1)));
                double unknownB = (((x2 - x1) * (y1 - y3)) - ((y2 - y1) * (x1 - x3))) /
                        (((y4 - y3) * (x2 - x1)) - ((x4 - x3) * (y2 - y1)));

                if (unknownA >= 0 && unknownA <= 1 && unknownB >= 0 && unknownB <= 1) {
                    return true;
                }
            }
        }

        return false;
    }
}
