package cnuphys.ced.cedview.central;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.geom.Point2D;

import cnuphys.bCNU.graphics.container.IContainer;
import cnuphys.ced.event.AccumulationManager;
import cnuphys.ced.event.data.AdcHit;
import cnuphys.ced.event.data.AdcHitList;
import cnuphys.ced.event.data.BMT;
import cnuphys.ced.event.data.CTOF;
import cnuphys.ced.event.data.DataDrawSupport;
import cnuphys.ced.event.data.BST;
import cnuphys.ced.event.data.BaseHit2;
import cnuphys.ced.event.data.BaseHit2List;
import cnuphys.ced.event.data.CND;
import cnuphys.ced.event.data.TdcAdcHit;
import cnuphys.ced.event.data.TdcAdcHitList;
import cnuphys.ced.geometry.BMTGeometry;
import cnuphys.ced.geometry.BSTGeometry;
import cnuphys.ced.geometry.BSTxyPanel;
import cnuphys.ced.geometry.bmt.BMTSectorItem;
import cnuphys.splot.plot.X11Colors;

public class CentralXYHitDrawer extends CentralHitDrawer {

	private static Color _baseColor = new Color(255, 0, 0, 60);

	// owner view
	private CentralXYView _view;

	public CentralXYHitDrawer(CentralXYView view) {
		super(view);
		_view = view;
	}

	@Override
	public String getName() {
		return "CentralXYHitDrawer";
	}

	// draw accumulated BST hits (panels)
	@Override
	protected void drawCNDAccumulatedHits(Graphics g, IContainer container) {
		int medianHit = AccumulationManager.getInstance().getMedianCNDCount();

		int cndData[][][] = AccumulationManager.getInstance().getAccumulatedCNDData();

		for (int sect0 = 0; sect0 < 24; sect0++) {
			for (int lay0 = 0; lay0 < 3; lay0++) {
				for (int order = 0; order < 2; order++) {

					int hitCount = cndData[sect0][lay0][order];

					CNDXYPolygon poly = _view.getCNDPolygon(sect0 + 1, lay0 + 1, order + 1);
					double fract = _view.getMedianSetting() * (((double) hitCount) / (1 + medianHit));
					Color color = AccumulationManager.getInstance().getColor(_view.getColorScaleModel(), fract);

					poly.draw(g, container, color, Color.black);

				}
			}
		}
	}

	// draw accumulated BST hits (panels)
	@Override
	protected void drawBSTAccumulatedHits(Graphics g, IContainer container) {
		// panels

		int medianHit = AccumulationManager.getInstance().getMedianBSTCount();

		// first index is layer 0..7, second is sector 0..23
		int bstData[][] = AccumulationManager.getInstance().getAccumulatedBSTData();

		for (int lay0 = 0; lay0 < 8; lay0++) {
			int supl0 = lay0 / 2;
			for (int sect0 = 0; sect0 < BSTGeometry.sectorsPerSuperlayer[supl0]; sect0++) {
				BSTxyPanel panel = CentralXYView.getPanel(lay0 + 1, sect0 + 1);

				if (panel != null) {
					int hitCount = bstData[lay0][sect0];

					double fract = _view.getMedianSetting() * (((double) hitCount) / (1 + medianHit));
					Color color = AccumulationManager.getInstance().getColor(_view.getColorScaleModel(), fract);
					_view.drawBSTPanel((Graphics2D) g, container, panel, color);

				} 
			}
		}
	}

	// draw CTOF accumulated hits
	@Override
	protected void drawCTOFAccumulatedHits(Graphics g, IContainer container) {
		int medianHit = AccumulationManager.getInstance().getMedianCTOFCount();

		int ctofData[] = AccumulationManager.getInstance().getAccumulatedCTOFData();

		for (int index = 0; index < 48; index++) {
			CTOFXYPolygon poly = _view.getCTOFPolygon(index + 1);
			if (poly != null) {
				int hitCount = ctofData[index];

				double fract = _view.getMedianSetting() * (((double) hitCount) / (1 + medianHit));

				Color color = AccumulationManager.getInstance().getColor(_view.getColorScaleModel(), fract);

				poly.draw(g, container, index + 1, color);
			}
		}
	}

	// only called in single event mode
	@Override
	protected void drawHitsSingleMode(Graphics g, IContainer container) {
		drawBSTHitsSingleMode(g, container);
		drawBMTHitsSingleMode(g, container);
		drawCTOFSingleHitsMode(g, container);
		drawCNDSingleHitsMode(g, container);
		drawCVTReconTraj(g, container);
	}
	

	// draw CTOF hits
	@Override
	protected void drawCNDSingleHitsMode(Graphics g, IContainer container) {

		CND cnd = CND.getInstance();

		int adcCount = cnd.getCountAdc();
		int tdcCount = cnd.getCountTdc();

		// tdc?
		if (tdcCount > 0) {
			for (int i = 0; i < tdcCount; i++) {
				int hsect = cnd.tdc_sect[i];
				int hlayer = cnd.tdc_layer[i];
				int hleftright = 1 + (cnd.tdc_order[i] % 2);

				CNDXYPolygon poly = _view.getCNDPolygon(hsect, hlayer, hleftright);

				poly.draw(g, container, Color.lightGray, Color.black);
			}
		}

		// adc?
		if (adcCount > 0) {
			for (int i = 0; i < adcCount; i++) {
				int hsect = cnd.adc_sect[i];
				int hlayer = cnd.adc_layer[i];
				int hleftright = 1 + (cnd.adc_order[i] % 2);

				CNDXYPolygon poly = _view.getCNDPolygon(hsect, hlayer, hleftright);

				Color color = cnd.adcColor(cnd.adc_ADC[i]);
				poly.draw(g, container, color, Color.black);

			}
		}

		// tdc again?
		if (tdcCount > 0) {
			for (int i = 0; i < tdcCount; i++) {
				int hsect = cnd.tdc_sect[i];
				int hlayer = cnd.tdc_layer[i];
				int hleftright = 1 + (cnd.tdc_order[i] % 2);

				CNDXYPolygon poly = _view.getCNDPolygon(hsect, hlayer, hleftright);

				g.setColor(Color.black);
				g.drawLine(poly.xpoints[0], poly.ypoints[0], poly.xpoints[2], poly.ypoints[2]);
				g.drawLine(poly.xpoints[1], poly.ypoints[1], poly.xpoints[3], poly.ypoints[3]);
			}
		}

	}

	// draw CTOF hits
	@Override
	protected void drawCTOFSingleHitsMode(Graphics g, IContainer container) {

		TdcAdcHitList hits = CTOF.getInstance().getHits();
		if ((hits != null) && !hits.isEmpty()) {
			for (TdcAdcHit hit : hits) {
				if (hit != null) {
					CTOFXYPolygon poly = _view.getCTOFPolygon(hit.component);
					if (poly != null) {
						Color color = hits.adcColor(hit);
						poly.draw(g, container, hit.component, color);
					}
				}
			}
		}

	}

	// draw BMT hits
	@Override
	protected void drawBMTHitsSingleMode(Graphics g, IContainer container) {

		Point pp = new Point();
		Point2D.Double wp = new Point2D.Double();

		AdcHitList hits = BMT.getInstance().getADCHits();
		if ((hits != null) && !hits.isEmpty()) {

//			Shape oldClip = g.getClip();
			Graphics2D g2 = (Graphics2D) g;

			for (AdcHit hit : hits) {
				if (hit != null) {
					BMTSectorItem bmtItem = _view.getBMTSectorItem(hit.sector, hit.layer);

					Polygon poly = bmtItem.getStripPolygon(container, hit.component);
					if (poly != null) {
						g.setColor(X11Colors.getX11Color("tan"));
						g.fillPolygon(poly);
						g.setColor(Color.red);
						g.drawPolygon(poly);
					}
				}
			}
		} // adc hits not empty

		if (_view.showADCHits() && (hits != null) && !hits.isEmpty()) {
			for (AdcHit hit : hits) {
				if (hit != null) {
					BMTSectorItem bmtItem = _view.getBMTSectorItem(hit.sector, hit.layer);

					if (bmtItem.isZLayer()) {

						Color color = hits.adcColor(hit);

						double phi = BMTGeometry.getGeometry().CRZStrip_GetPhi(hit.sector, hit.layer, hit.component);

						double rad = bmtItem.getInnerRadius() + BMTSectorItem.FAKEWIDTH / 2.;
						wp.x = rad * Math.cos(phi);
						wp.y = rad * Math.sin(phi);
						container.worldToLocal(pp, wp);
						hit.setLocation(pp);
						DataDrawSupport.drawAdcHit(g, pp, color);

						// SymbolDraw.drawX(g2, pp.x, pp.y, 4, Color.black);
					}
				}
			}
		} // adc hits not empty

		if (_view.showReconHits()) {
			BaseHit2List recHits = BMT.getInstance().getRecHits();
			if (recHits != null) {
				// System.err.println("NUM RECON BMT HITS: " +
				// recHits.getCount());

				if (recHits.count() > 0) {
					for (BaseHit2 bhit2 : recHits) {
						BMTSectorItem bmtItem = _view.getBMTSectorItem(bhit2.sector, bhit2.layer);
						if (bmtItem.isZLayer()) {

							double phi = BMTGeometry.getGeometry().CRZStrip_GetPhi(bhit2.sector, bhit2.layer,
									bhit2.component);

							double rad = bmtItem.getInnerRadius() + BMTSectorItem.FAKEWIDTH / 2.;
							wp.x = rad * Math.cos(phi);
							wp.y = rad * Math.sin(phi);
							container.worldToLocal(pp, wp);

							bhit2.setLocation(pp);
							DataDrawSupport.drawReconHit(g, pp);
						}
					}
				}
			}
		}
	}

	// draw BST hits single event mode
	@Override
	protected void drawBSTHitsSingleMode(Graphics g, IContainer container) {

		AdcHitList hits = BST.getInstance().getHits();
		if ((hits != null) && !hits.isEmpty()) {

//			Shape oldClip = g.getClip();
			Graphics2D g2 = (Graphics2D) g;

			for (AdcHit hit : hits) {
				if (hit != null) {

					BSTxyPanel panel = CentralXYView.getPanel(hit.layer, hit.sector);

					if (panel != null) {
						_view.drawBSTPanel(g2, container, panel, _baseColor);
						_view.drawBSTPanel(g2, container, panel, hits.adcColor(hit));
						// _view.drawBSTPanel(g2, container, panel, Color.red);
					}

				}
			}
		}
	}

}
