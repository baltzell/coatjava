package cnuphys.ced.ced3d;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JPanel;

import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.awt.GLJPanel;

import cnuphys.bCNU.component.checkboxarray.CheckBoxArray;
import cnuphys.bCNU.dialog.VerticalFlowLayout;
import cnuphys.bCNU.graphics.GraphicsUtilities;
import cnuphys.bCNU.graphics.component.CommonBorder;
import cnuphys.bCNU.util.Fonts;
import cnuphys.bCNU.util.PrintUtilities;
import cnuphys.bCNU.view.VirtualView;
import cnuphys.ced.ced3d.view.CedView3D;
import cnuphys.ced.clasio.ClasIoEventManager;
import cnuphys.ced.frame.Ced;
import bCNU3D.Panel3D;

public class CedPanel3D extends Panel3D {

	// labels for the check box
	public static final String SHOW_VOLUMES = "Volumes";
	public static final String SHOW_TRUTH = "Truth";
	public static final String SHOW_DC = "DC";
	public static final String SHOW_ECAL = "ECAL";
	public static final String SHOW_PCAL = "PCAL";
	public static final String SHOW_FTOF = "FTOF";
//	public static final String SHOW_SIM_SDOCA = "SDOCA";
//	public static final String SHOW_TB_DOCA = "TB DOCA";
	public static final String SHOW_RECON_FTOF = "Rec-FTOF";
	public static final String SHOW_RECON_CROSSES = "Crosses";
	public static final String SHOW_SVT = "SVT";
	public static final String SHOW_SVT_LAYER_1 = "SVT Layer 1";
	public static final String SHOW_SVT_LAYER_2 = "SVT Layer 2";
	public static final String SHOW_SVT_LAYER_3 = "SVT Layer 3";
	public static final String SHOW_SVT_LAYER_4 = "SVT Layer 4";
	public static final String SHOW_SVT_LAYER_5 = "SVT Layer 5";
	public static final String SHOW_SVT_LAYER_6 = "SVT Layer 6";
	public static final String SHOW_SVT_LAYER_7 = "SVT Layer 7";
	public static final String SHOW_SVT_LAYER_8 = "SVT Layer 8";
	public static final String SHOW_SVT_HITS = "SVT Hits";

	public static final String SHOW_SECTOR_1 = "Sector 1";
	public static final String SHOW_SECTOR_2 = "Sector 2";
	public static final String SHOW_SECTOR_3 = "Sector 3";
	public static final String SHOW_SECTOR_4 = "Sector 4";
	public static final String SHOW_SECTOR_5 = "Sector 5";
	public static final String SHOW_SECTOR_6 = "Sector 6";

	public static final String SHOW_CTOF = "CTOF";

	public static final String SHOW_CND = "CND";
	public static final String SHOW_CND_LAYER_1 = "CND Layer 1";
	public static final String SHOW_CND_LAYER_2 = "CND Layer 2";
	public static final String SHOW_CND_LAYER_3 = "CND Layer 3";
	
	public static final String SHOW_TB_CROSS = "TB Cross";
	public static final String SHOW_HB_CROSS = "HB Cross";
	public static final String SHOW_TB_TRACK = "TB Track";
	public static final String SHOW_HB_TRACK = "HB Track";

	public static final String SHOW_COSMIC = "Cosmics";

	// alpha value for volumes
	protected int volumeAlpha = 28;

	// Check box array
	protected CheckBoxArray _checkBoxArray;

	// show what particles are present
	private PIDLegend _pidLegend;

	// alpha slider for volume alphas
	private AlphaSlider _volumeAlphaSlider;

	// display array labels
	private String _cbaLabels[];
	
	private CedView3D _view;

	/*
	 * The panel that holds the 3D objects
	 * 
	 * @param angleX the initial x rotation angle in degrees
	 * 
	 * @param angleY the initial y rotation angle in degrees
	 * 
	 * @param angleZ the initial z rotation angle in degrees
	 * 
	 * @param xdist move viewpoint left/right
	 * 
	 * @param ydist move viewpoint up/down
	 * 
	 * @param zdist the initial viewer z distance should be negative
	 */
	public CedPanel3D(CedView3D view, float angleX, float angleY, float angleZ, float xDist,
			float yDist, float zDist, String... cbaLabels) {
		super(angleX, angleY, angleZ, xDist, yDist, zDist);

		_view = view;
		_cbaLabels = cbaLabels;

		gljpanel.setBorder(new CommonBorder());
		final GLJPanel gljp = gljpanel;

		addEast();
		addNorth();

		ActionListener al = new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				refresh();
				gljp.requestFocus();
			}

		};
		for (String s : _cbaLabels) {
			AbstractButton ab = _checkBoxArray.getButton(s);
			ab.setFont(Fonts.smallFont);
			ab.setSelected(true);

//			if (!SHOW_SIM_SDOCA.equals(s)) {
//				ab.setSelected(true);
//			}
			ab.addActionListener(al);
		}
		fixSize();
		
		enableSVTOuterLayers();
	}
	
	@Override
	public void display(GLAutoDrawable drawable) {
		if (VirtualView.getInstance().isViewVisible(_view)) {
			super.display(drawable);
		}
		else {
			System.err.println("SKIPPED");
		}
	}
	

	// add north panel
	private void addNorth() {
		JPanel np = new JPanel();
		np.setLayout(new BorderLayout(20, 0));
		
		JButton nextEvent;
		ActionListener al = new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				if (ClasIoEventManager.getInstance().isNextOK()) {
					ClasIoEventManager.getInstance().getNextEvent();
				} else {
					Toolkit.getDefaultToolkit().beep();
				}
			}

		};
		nextEvent = new JButton("Next");
		nextEvent.setToolTipText("Next Event");
		nextEvent.addActionListener(al);
		GraphicsUtilities.setSizeMini(nextEvent);
		np.add(nextEvent, BorderLayout.WEST);


		_pidLegend = new PIDLegend(this);
		_volumeAlphaSlider = new AlphaSlider(this, "Volume alpha");
		np.add(_volumeAlphaSlider, BorderLayout.EAST);
		np.add(_pidLegend, BorderLayout.CENTER);
		add(np, BorderLayout.NORTH);
	}

	// add eastern panel
	private void addEast() {
		JPanel ep = new JPanel();
		ep.setLayout(new VerticalFlowLayout());

		ep.add(new KeyboardLegend());
		_checkBoxArray = new CheckBoxArray(2, 4, 4, _cbaLabels);
		_checkBoxArray.setBorder(new CommonBorder());
		ep.add(_checkBoxArray);

		add(ep, BorderLayout.EAST);
	}

	@Override
	public void refresh() {
		super.refresh();
		_pidLegend.repaint();
	}

	/**
	 * Print the panel.
	 */
	@Override
	public void print() {
		PrintUtilities.printComponent(this);
	}

	/**
	 * Snapshot of the panel.
	 */
	@Override
	public void snapshot() {
		GraphicsUtilities.saveAsPng(this);
	}

	// a fixed fraction of the screen
	private void fixSize() {
		Dimension d = GraphicsUtilities.screenFraction(0.60);
		d.width = d.height;
		gljpanel.setPreferredSize(d);
	}


	/**
	 * Get the alpha for volume drawing
	 * 
	 * @return the alpha for volume drawing
	 */
	public int getVolumeAlpha() {
		return _volumeAlphaSlider.getAlpha();
	}

	/**
	 * Get one of the display buttons
	 * 
	 * @param label
	 *            the button label
	 * @return the button or null on failure
	 */
	public AbstractButton getDisplayButton(String label) {
		if (_checkBoxArray == null) {
			return null;
		}
		return _checkBoxArray.getButton(label);
	}

	/**
	 * Get all the display buttons in an array
	 * 
	 * @return all the display buttons
	 */
	public AbstractButton[] gatAllDisplayButtons() {
		if ((_checkBoxArray == null) || (_cbaLabels == null)) {
			return null;
		}

		if (_cbaLabels.length < 1) {
			return null;
		}

		AbstractButton buttons[] = new AbstractButton[_cbaLabels.length];

		for (int i = 0; i < _cbaLabels.length; i++) {
			buttons[i] = _checkBoxArray.getButton(_cbaLabels[i]);
		}

		return buttons;
	}

	/**
	 * Check if a feature should be drawn
	 * 
	 * @param label
	 *            the label for the check box on the option array
	 * @return <code>true</code> if the feature should be drawn
	 */
	private boolean show(String label) {
		AbstractButton ab = _checkBoxArray.getButton(label);
		return (ab == null) ? false : ab.isSelected();
	}
	
	public void enableLabel(String label, boolean enabled) {
		AbstractButton ab = _checkBoxArray.getButton(label);
		if (ab != null) {
			ab.setEnabled(enabled);
		}
	}
	
	public void enableSVTOuterLayers() {
		boolean oldGeo = Ced.getCed().useOldSVTGeometry();
		enableLabel(SHOW_SVT_LAYER_7, oldGeo);
		enableLabel(SHOW_SVT_LAYER_8, oldGeo);
	}

	/**
	 * Show ECAL?
	 * @return <code>true</code> if we are to show ECAL
	 */
	public boolean showECAL() {
		return show(CedPanel3D.SHOW_ECAL);
	}

	/**
	 * Show PCAL?
	 * @return <code>true</code> if we are to show PCAL
	 */
	public boolean showPCAL() {
		return show(CedPanel3D.SHOW_PCAL);
	}

	/**
	 * Show forward TOF?
	 * @return <code>true</code> if we are to show FTOF
	 */
	public boolean showFTOF() {
		return show(CedPanel3D.SHOW_FTOF);
	}
	
	/**
	 * Show CTOF?
	 * @return <code>true</code> if we are to show CTOF
	 */
	public boolean showCTOF() {
		return show(CedPanel3D.SHOW_CTOF);
	}


	/**
	 * Show SVT?
	 * @return <code>true</code> if we are to show SVT
	 */
	public boolean showSVT() {
		return show(CedPanel3D.SHOW_SVT);
	}

	
	/**
	 * Show CND?
	 * @return <code>true</code> if we are to show CND
	 */
	public boolean showCND() {
		return show(CedPanel3D.SHOW_CND);
	}

	/**
	 * Show CND Layer 1?
	 * @return <code>true</code> if we are to show CND Layer 1
	 */
	public boolean showCNDLayer1() {
		return showCND() && show(CedPanel3D.SHOW_CND_LAYER_1);
	}

	/**
	 * Show CND Layer 2?
	 * @return <code>true</code> if we are to show CND Layer 2
	 */
	public boolean showCNDLayer2() {
		return showCND() && show(CedPanel3D.SHOW_CND_LAYER_2);
	}

	/**
	 * Show CND Layer 3?
	 * @return <code>true</code> if we are to show CND Layer 3
	 */
	public boolean showCNDLayer3() {
		return showCND() && show(CedPanel3D.SHOW_CND_LAYER_3);
	}
	
	/**
	 * Show reconstructed Crosses?
	 * @return <code>true</code> if we are to show reconstructed crosses
	 */
	public boolean showReconCrosses() {
		return show(CedPanel3D.SHOW_RECON_CROSSES);
	}

	/**
	 * Show reconstructed FTOF?
	 * @return <code>true</code> if we are to show reconstructed ftof
	 */
	public boolean showReconFTOF() {
		return show(CedPanel3D.SHOW_RECON_FTOF);
	}

	/**
	 * Show time based track?
	 * @return <code>true</code> if we are to show time based track
	 */
	public boolean showTBTrack() {
		return show(CedPanel3D.SHOW_TB_TRACK);
	}
	
	/**
	 * Show hit based track?
	 * @return <code>true</code> if we are to show hit based track
	 */
	public boolean showHBTrack() {
		return show(CedPanel3D.SHOW_HB_TRACK);
	}


	/**
	 * Show time based cross?
	 * @return <code>true</code> if we are to show time based cross
	 */
	public boolean showTBCross() {
		return show(CedPanel3D.SHOW_TB_CROSS);
	}
	
	/**
	 * Show hit based cross?
	 * @return <code>true</code> if we are to show hit based cross
	 */
	public boolean showHBCross() {
		return show(CedPanel3D.SHOW_HB_CROSS);
	}

	/**
	 * Show SVT Hits?
	 * @return <code>true</code> if we are to show SVT Hits
	 */
	public boolean showSVTHits() {
		return show(CedPanel3D.SHOW_SVT_HITS);
	}
	
	/**
	 * Show DC?
	 * @return <code>true</code> if we are to show DC
	 */
	public boolean showDC() {
		return show(CedPanel3D.SHOW_DC);
	}
	
	/**
	 * Show SVT Layer 1?
	 * @return <code>true</code> if we are to show SVT Layer 1
	 */
	public boolean showSVTLayer1() {
		return showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_1);
	}

	/**
	 * Show SVT Layer 2?
	 * @return <code>true</code> if we are to show SVT Layer 2
	 */
	public boolean showSVTLayer2() {
		return showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_2);
	}

	/**
	 * Show SVT Layer 3?
	 * @return <code>true</code> if we are to show SVT Layer 3
	 */
	public boolean showSVTLayer3() {
		return showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_3);
	}

	/**
	 * Show SVT Layer 4?
	 * @return <code>true</code> if we are to show SVT Layer 4
	 */
	public boolean showSVTLayer4() {
		return showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_4);
	}

	/**
	 * Show SVT Layer 5?
	 * @return <code>true</code> if we are to show SVT Layer 5
	 */
	public boolean showSVTLayer5() {
		return showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_5);
	}

	/**
	 * Show SVT Layer 6?
	 * @return <code>true</code> if we are to show SVT Layer 6
	 */
	public boolean showSVTLayer6() {
		return showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_6);
	}

	/**
	 * Show SVT Layer 7?
	 * @return <code>true</code> if we are to show SVT Layer 7
	 */
	public boolean showSVTLayer7() {
		boolean oldSVTGeometry = Ced.getCed().useOldSVTGeometry();
		return oldSVTGeometry && showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_7);
	}

	/**
	 * Show SVT Layer 8?
	 * @return <code>true</code> if we are to show SVT Layer 8
	 */
	public boolean showSVTLayer8() {
		boolean oldSVTGeometry = Ced.getCed().useOldSVTGeometry();
		return oldSVTGeometry && showSVT() && show(CedPanel3D.SHOW_SVT_LAYER_8);
	}

	/**
	 * Show sector 1?
	 * @return <code>true</code> if we are to show sector 1
	 */
	public boolean showSector1() {
		return show(CedPanel3D.SHOW_SECTOR_1);
	}

	/**
	 * Show sector 2?
	 * @return <code>true</code> if we are to show sector 2
	 */
	public boolean showSector2() {
		return show(CedPanel3D.SHOW_SECTOR_2);
	}

	/**
	 * Show sector 3?
	 * @return <code>true</code> if we are to show sector 3
	 */
	public boolean showSector3() {
		return show(CedPanel3D.SHOW_SECTOR_3);
	}

	/**
	 * Show sector 4?
	 * @return <code>true</code> if we are to show sector 4
	 */
	public boolean showSector4() {
		return show(CedPanel3D.SHOW_SECTOR_4);
	}

	/**
	 * Show sector 5?
	 * @return <code>true</code> if we are to show sector 5
	 */
	public boolean showSector5() {
		return show(CedPanel3D.SHOW_SECTOR_5);
	}

	/**
	 * Show sector 6?
	 * @return <code>true</code> if we are to show sector 6
	 */
	public boolean showSector6() {
		return show(CedPanel3D.SHOW_SECTOR_6);
	}
	
	/**
	 * Show Cosmics?
	 * @return <code>true</code> if we are to show Cosmics
	 */
	public boolean showCosmics() {
		return show(CedPanel3D.SHOW_COSMIC);
	}



	/**
	 * Show we show the 1-based sector?
	 * @param sector the sector [1..6]
	 * @return <code>true</code> if we are to show the sector
	 */
	public boolean showSector(int sector) {
		switch (sector) {
		case 1:
			return showSector1();
		case 2:
			return showSector2();
		case 3:
			return showSector3();
		case 4:
			return showSector4();
		case 5:
			return showSector5();
		case 6:
			return showSector6();
		}
		return false;
	}

	/** 
	 * Show MC truth?
	 * @return <code>true</code> if we are to show simulation truth
	 */
	public boolean showMCTruth() {
		return show(CedPanel3D.SHOW_TRUTH);
	}

	/** 
	 * Show Volumes?
	 * @return <code>true</code> if we are to show volumes
	 */
	public boolean showVolumes() {
		return show(CedPanel3D.SHOW_VOLUMES);
	}

}
