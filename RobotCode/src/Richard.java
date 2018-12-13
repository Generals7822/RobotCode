
import java.awt.Color;
import java.awt.Graphics;

import javax.swing.JFrame;
import javax.swing.JPanel;
//1080*X

public class Richard extends JPanel {

	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		double frequency = .2;
		int size = 1000;
		double rSize = 2.5 / size;
		for (double initB = 1.25; initB >= -1.25; initB -= rSize) {
			for (double initA = -2; initA <= .5; initA += rSize) {
				double a = initA;
				double b = initB;
				int i;
				for (i = 0; i <= 1000; i++) {
					double keepA = a;
					a = a * a - b * b;
					b = 2 * keepA * b;
					a += initA;
					b += initB;
					if (a > 2 || b > 2 || a < -2 || b < -2) {
						break;
					}
				}
				if (i == 1001) {
					g.setColor(Color.BLACK);
					g.drawLine((int) (initA * size / 2.5 + 2 * size / 2.5),
							(int) (initB * size / 2.5 + 1.25 * size / 2.5), (int) (initA * size / 2.5 + 2 * size / 2.5),
							(int) (initB * size / 2.5 + 1.25 * size / 2.5));
				} else {
					if (i == 0) {
						i = 1;
					}
					int red = (int) (Math.sin(frequency * i) * 127 + 128);
					int green = (int) (Math.sin(frequency * i + 2 * Math.PI / 3) * 127 + 128);
					int blue = (int) (Math.sin(frequency * i + 4 * Math.PI / 3) * 127 + 128);
					g.setColor(new Color(red, green, blue));
					g.drawLine((int) (initA * size / 2.5 + 2 * size / 2.5),
							(int) (initB * size / 2.5 + 1.25 * size / 2.5), (int) (initA * size / 2.5 + 2 * size / 2.5),
							(int) (initB * size / 2.5 + 1.25 * size / 2.5));

				}
			}
		}
		g.setColor(Color.BLACK);
	}

	public static void main(String[] args) {
		Richard panel = new Richard();
		JFrame application = new JFrame();
		application.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		application.add(panel);
		application.setSize(1080, 1080);
		application.setVisible(true);
	}

}
