package cnuphys.ced.event.data;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JPanel;
import cnuphys.bCNU.dialog.DialogUtilities;
import cnuphys.splot.pdata.HistoData;

public class DefineHistoDialog extends JDialog implements ActionListener, PropertyChangeListener {
	
	private JButton _okButton;
	private JButton _cancelButton;
	private int _reason = DialogUtilities.CANCEL_RESPONSE;
	private HistoPanel _histoPanel;

	public DefineHistoDialog() {
		setTitle("Define a Histogram");
		setModal(false);
		setLayout(new BorderLayout(4, 4));
		
		_histoPanel = new HistoPanel("Select a Variable");
		add(_histoPanel, BorderLayout.CENTER);

		_histoPanel.getSelectPanel().addPropertyChangeListener(this);
		
		addSouth();
		pack();
		DialogUtilities.centerDialog(this);
	}
	
	//add the buttons
	private void addSouth(){
		JPanel sp = new JPanel();
		sp.setLayout(new FlowLayout(FlowLayout.CENTER, 200, 10));
		
		_okButton = new JButton("  OK  ");
		_okButton.setEnabled(false);
		_cancelButton = new JButton("Cancel");
		
		_okButton.addActionListener(this);
		_cancelButton.addActionListener(this);
		
		sp.add(_okButton);
		sp.add(_cancelButton);
		add(sp, BorderLayout.SOUTH);
	}
	
	/**
	 * Get the reason the dialog closed
	 * @return the reason the dialog closed
	 */
	public int getReason() {
		return _reason;
	}
	
	/**
	 * Return a HistoData ready for filling if the user hit ok
	 * @return a HistoData or <code>null</code>.
	 */
	public HistoData getHistoData() {
		if (_reason == DialogUtilities.OK_RESPONSE) {
			return _histoPanel.getHistoData();
		}
		return null;
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		Object o = e.getSource();
		if (o == _okButton) {
			_reason = DialogUtilities.OK_RESPONSE;
			setVisible(false);
		}
		else if (o == _cancelButton) {
			_reason = DialogUtilities.CANCEL_RESPONSE;
			setVisible(false);
		}
	}

	
	@Override
	public void propertyChange(PropertyChangeEvent evt) {
		Object o = evt.getSource();
		String prop = evt.getPropertyName();
		if ((o == _histoPanel.getSelectPanel()) && prop.equals("newname")) {
			String fn = (String)(evt.getNewValue());
			boolean valid = ((fn != null) && (fn.length() > 4) && fn.contains(":") && fn.contains("."));
			_okButton.setEnabled(valid);
		}	
	}
	
	public static void main(String arg[]) {
		DefineHistoDialog dialog = new DefineHistoDialog();
		dialog.setVisible(true);
		int reason = dialog.getReason();
		if (reason == DialogUtilities.OK_RESPONSE) {
			System.err.println("OK");
		}
		else {
			System.err.println("CANCEL");
		}
		
		System.exit(0);
	}

}
