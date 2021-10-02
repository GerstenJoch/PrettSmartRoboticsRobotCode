package GerstenJoch;

import javax.swing.JFrame;

public class GameFrame extends JFrame{
	GameFrame(){
		JFrame frame = new JFrame();
		frame.add(new GamePanel());
		frame.setTitle("Snake");
		frame.setDefaultCloseOperation(EXIT_ON_CLOSE);
		frame.setResizable(false);
		frame.pack();
		frame.setVisible(true);
		frame.setLocationRelativeTo(null);
		
	}
}
