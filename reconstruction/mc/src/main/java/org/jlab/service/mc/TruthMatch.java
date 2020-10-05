package org.jlab.service.mc;

import org.jlab.clas.reco.ReconstructionEngine;
import org.jlab.detector.base.DetectorType;
import org.jlab.io.base.DataBank;
import org.jlab.io.base.DataEvent;
import java.util.*;

/**
 *
 * @author rafopar
 *
 */
/**
 *
 * Another approach for making truth matching that will include neutral
 * particles as well The "TruthMatching" class works well for charged particles,
 * but for neutrals it requires to introduce some ad-hoc functions. So this is
 * an attempt to make it to look more general.
 */
public class TruthMatch extends ReconstructionEngine {

    /**
     * We introduce this variable which will show whether there is a neutral
     * particle in the MC::Particle bank. In case all particles are charged then
     * we don't need to use all secondary hits, but instead we will use hits
     * that are created directly from the MC particle
     */
    private boolean hasNeutral;

    public TruthMatch() {
        super("TruthMatch", "Rafo", "0.0");
        hasNeutral = false;
    }

    @Override
    public boolean init() {
        return true;
    }

    @Override
    public boolean processDataEvent(DataEvent event) {

        System.out.println("========================================= Event "
                + event.getBank("RUN::config").getInt("event", 0) + " =========================================");

        // check if the event contains the MC banks
        if (event.hasBank("MC::True") == false) {
            System.err.print(" [ WARNING, TruthMatching ]: no MC::True bank found");
            if (event.hasBank("RUN::config")) {
                System.err.print(" ******* EVENT " + event.getBank("RUN::config").getInt("event", 0));
            }
            System.err.println();
            return false;
        }

        if (event.hasBank("MC::Particle") == false) {
            System.err.println(" [ WARNING, TruthMatching ]: no MC::Particle bank found");
            return false;
        }

        /**
         * ********************************************************
         * The 1st thing, let's load MC particles
         * ********************************************************
         */
        Map<Short, MCPart> mcp = getMCparticles(event.getBank("MC::Particle"));

        /**
         * ********************************************************
         * Now let's get True hits from the MC::True bank
         * ********************************************************
         */
        Map<Byte, Map<Integer, MCHit>> mchits = getMCHits(event.getBank("MC::True"), mcp);

        /**
         * Now let's get ECal Rec hits. The RecHit object should, has the -
         * hitID (the same as the hitn in MCHit) - cID : clusterId of the
         * cluster to which the hit belogs to - pindex : pindex of the particle
         * that the current cluster belongs to.
         */
        Map< Short, List<RecHit>> ecalHits = getECalHits(event, mchits.get((byte) DetectorType.ECAL.getDetectorId()));
        List<RecCluster> ecalClusters = getECalClusters(event);

        /**
         * Getting FT Hits and clusters
         */
        Map< Short, List<RecHit>> ftCalHits = getFTCalHits(event, mchits.get((byte) DetectorType.FTCAL.getDetectorId()));
        List<RecCluster> ftCalClusters = getFTCalClusters(event);

        /**
         * Getting CND Hits and Clusters
         */
        Map<Short, List<RecHit>> cndHits = getCNDHits(event, mchits.get((byte) DetectorType.CND.getDetectorId()));
        List<RecCluster> cndClusters = getCNDClusters(event);

        /**
         * Getting CTOF Hits and Clusters
         */
        Map<Short, List<RecHit>> ctofHits = getCTOFHits(event, mchits.get((byte) DetectorType.CND.getDetectorId()));
        List<RecCluster> ctofClusters = getCTOFClusters(event);

        /**
         * Matchingg clusters to MCParticles
         */
        MatchClasters(ecalClusters, ecalHits, mchits.get((byte) DetectorType.ECAL.getDetectorId()));
        MatchClasters(ftCalClusters, ftCalHits, mchits.get((byte) DetectorType.FTCAL.getDetectorId()));
        MatchClasters(cndClusters, cndHits, mchits.get((byte) DetectorType.CND.getDetectorId()));
        MatchClasters(ctofClusters, ctofHits, mchits.get((byte) DetectorType.CTOF.getDetectorId()));

        /**
         * Adding all clusters together
         */
        List<RecCluster> allCls = new ArrayList<>();

        /**
         * Adding ECal clusters
         */
        if (ecalClusters != null) {
            allCls.addAll(ecalClusters);
        }
        /**
         * Adding FTCal clusters
         */
        if (ftCalClusters != null) {
            allCls.addAll(ftCalClusters);
        }

        /**
         * Adding CND clusters
         */
        if (cndClusters != null) {
            allCls.addAll(cndClusters);
        }

        /**
         * Adding CTOF clusters
         */
        if (cndClusters != null) {
            allCls.addAll(ctofClusters);
        }

        Map<Short, List<RecCluster>> clsPerMCp = mapClustersToMCParticles(mcp.keySet(), allCls);

        //PrintClsPerMc(clsPerMCp);
        List<MCRecMatch> MCRecMatches = MakeMCRecMatch(mcp, clsPerMCp);

        bankWriter(event, MCRecMatches, allCls);

        //PrintRecMatches(MCRecMatches);
        /**
         *
         *
         */
        return true;
    }

    /**
     * ************************************************************************
     * Defining objects that will be needed int the matching process
     * ************************************************************************
     */
    // MCParticle from MC::Particle bank
    class MCPart {

        public int id;      // index of the MC particle (it should correspond of tid/otid
        // (still needs to be defined which one) in MC::True)

        public int pid;     // PDG ID code
    }

    // True hit information from the MC::True banl
    class MCHit {

        public int pid;      // MC particle id (pdg code)
        public int otid;     // id of the original (gernerated) particle that eventually caused the hit
        public int hitn;     // Hit id: it corresponds to the position of rec hits.
        public byte detector; // Detector code descriptor
    }

    // RecHit object
    class RecHit {

        public int id;          // ID: This is corresponding ADC row number in the ADC bank.
        public short pindex;      // pindex
        public short cid;         // clusterID
        public byte detector;     // Detector specifier

        @Override
        public String toString() {
            String str = "***** RecHit object ******\n";

            str += "* id = " + String.valueOf(id) + "\n";
            str += "* pindex = " + String.valueOf(pindex) + "\n";
            str += "* cid = " + String.valueOf(cid) + "\n";
            str += "* detector = " + String.valueOf(cid) + "\n";
            str += "***** End of RecHit object ******\n";
            return str;
        }

    }

    class RecCluster {

        public RecCluster() {
            nHitMatched = 0;
            mcotid = -1;
        }
        public short id;            // cluster id
        public short mcotid;        // mc track id
        public short rectid;        // rec track id
        public short pindex;        // index of the rec particle
        public int nHitMatched;     // number of hits MC matched
        public short size;          // number of hits
        public byte detector;
        public byte layer;
        public byte superlayer;
        public byte sector;
        public float energy;

        @Override
        public String toString() {

            String str = "***** RecCluster Object *****\n";

            str += "id = " + String.valueOf(id) + "\n";
            str += "mcotid = " + String.valueOf(mcotid) + "\n";
            str += "rectid = " + String.valueOf(rectid) + "\n";
            str += "pindex = " + String.valueOf(pindex) + "\n";
            str += "nHitMatched = " + String.valueOf(nHitMatched) + "\n";
            str += "size = " + String.valueOf(size) + "\n";
            str += "detector = " + String.valueOf(detector) + "\n";
            str += "layer = " + String.valueOf(layer) + "\n";
            str += "superlayer = " + String.valueOf(superlayer) + "\n";
            str += "sector = " + String.valueOf(sector) + "\n";
            str += " ***** All info about the RecCluster Object is printed \n \n";

            return str;
        }
    }

    class MCRecMatch {

        public MCRecMatch() {
            id = -1;
            nclusters = -1;
            nClsInRec = -1;
            frac = (float) -1.0;
            tid = -1;
            pindex = -1;
        }
        public short id;        // MC particle id
        public short pindex;    // pindex index of the rec particle in the REC::Particle bank
        public short nclusters; // number of matched clusters
        public short nClsInRec; // number of clusters used in reconstruction
        public float frac;      // fraction of clusters used
        public short tid;       // reconstructed track id
        public byte reconstructable;       // reconstructed track id

    }

    /**
     * Some constatnts
     */
    private final int BMTID = 1;
    private final int BSTID = 2;
    private final int DCID = 6;
    private final int ECALID = 7;
    private final int CNDID = 3;
    private final int CTOFID = 4;
    private final int FTCALID = 10;

    private final int PHOTON_ID = 22;
    private final int NEUTRON_ID = 2112;

    /**
     * ************************************************************************
     * Defining utility functions
     * ************************************************************************
     */
    /**
     *
     * @param MC::Particle bank
     *
     * @return map of MCpart objects, where the Key is the index of the MC
     * particle in the MC::Particle bank
     */
    Map<Short, MCPart> getMCparticles(DataBank mcpart) {

        Map<Short, MCPart> mcp = new HashMap<>();

        for (int i = 0; i < mcpart.rows(); i++) {

            MCPart curPart = new MCPart();

            curPart.id = i;
            curPart.pid = mcpart.getInt("pid", i);

            /**
             * Check if there is a neutral particle in the MC::Particle
             */
            if (curPart.pid == 22 /*photon*/ || curPart.pid == 2112/*Neutron*/) {
                hasNeutral = true;
            }

            mcp.put((short) (i), curPart);
        }
        return mcp;

    }

    Map<Byte, Map<Integer, MCHit>> getMCHits(DataBank mctrue, Map<Short, MCPart> mcp) {

        Map<Byte, Map<Integer, MCHit>> dmchits = new HashMap<>();

        for (int i = 0; i < mctrue.rows(); i++) {
            MCHit hit = new MCHit();
            hit.pid = mctrue.getInt("pid", i);
            hit.otid = mctrue.getInt("otid", i) - 1;
            hit.hitn = mctrue.getInt("hitn", i) - 1;
            hit.detector = mctrue.getByte("detector", i);

            int tid = mctrue.getInt("tid", i) - 1;  // in MC::True tid starts 
            //  from one, so subtracting 1 to match to the row number in the MC::Particle bank
            int mtid = mctrue.getInt("mtid", i);

            /**
             * In the original version of Truth Matching, before adding the
             * MC::True hit into the returned hit list, it checks whether the
             * tid (track id) of a hit is the id of one of MCPartciles
             * (generated particles), i.e. whether the hit is directly produced
             * from the originally generated MCParticle. This is good for
             * tracking detectors, as many hits unrelated (not directly related)
             * to the track, will not be followed.
             *
             * In calorimeters however lots of particle creation and killing is
             * ongoing, and only small fraction of hits (if any) is actually
             * created directly from the original particle, but rather are
             * created from daughters/granddaughters of the MCParticle.
             *
             * So for this purpose we have to keep all hits for calorimeters, if
             * the original particle is either photon or neutron, otherwise hits
             * in the ECAL or FTCak will not be kept.
             *
             * For tracking detectors we will throw non-direct hits from MC
             * particle
             *
             */
            if (hit.detector != (byte) DetectorType.ECAL.getDetectorId() && hit.detector != (byte) DetectorType.FTCAL.getDetectorId()
                    && mcp.get((short) tid) == null) {
                continue;
            } else if (mcp.get((short) (hit.otid)).pid != 22 && mcp.get((short) (hit.otid)).pid != 2112) {
                continue;
            }

            /**
             * We can check whether the particle or it's mother is original
             * particle. This can reduce the number of hits. Subject further
             * studies... if( mcp.get( (short) tid ) == null && mcp.get( (short)
             * mtid ) == null ) continue;
             */
            if (dmchits.get(hit.detector) == null) {
                dmchits.put(hit.detector, new HashMap<>());
            }
            dmchits.get(hit.detector).put(hit.hitn, hit);

        }
        return dmchits;
    }

    /**
     *
     * @param event DataEvent
     * @param mchitsInECal MCHits in ECal
     * @return Map<clusterID, List<RecHit>>, Map, where the Key is the
     * clusterID, and the value is a list of hits having the same clusterID
     */
    Map< Short, List<RecHit>> getECalHits(DataEvent event, Map<Integer, MCHit> mchitsInECal) {

        /**
         * We need two banks to be present in the event: ECAL::hits and
         * REC::Calorimeter "id" and "cid (clusterId)" will be obtained from the
         * ECAL::hits bank, but in order to find the pindex of particle we need
         * to loop over the REC::Calorimeter bank and create a map
         * <cId, pindex>.
         */
        Map< Short, List<RecHit>> recHits = new HashMap<>();
        if (mchitsInECal == null) {
            /**
             * In case if no MC hit present in the ECal, then don't proceed, as
             * we need only hits that are associated to an MC hit
             */
            return recHits;
        }

        /**
         * Check if two necessary banks exist otherwise will return null
         */
        if (event.hasBank("ECAL::hits") == false) {
            return null;
        }

        if (event.hasBank("REC::Calorimeter") == false) {
            return null;
        }

        Map<Short, Short> clId2Pindex = new HashMap<>();
        DataBank RecCalBank = event.getBank("REC::Calorimeter");

        for (int ical = 0; ical < RecCalBank.rows(); ical++) {

            Short pindex = RecCalBank.getShort("pindex", ical);
            Short index = RecCalBank.getShort("index", ical);

            clId2Pindex.put(index, pindex);
        }

        DataBank hitsBank = event.getBank("ECAL::hits");

//        for (Short theKey : clId2Pindex.keySet()) {
//            System.out.println("The Key of clId2Pindex is " + theKey + "The value is " + clId2Pindex.get(theKey));
//        }
        for (int ihit = 0; ihit < hitsBank.rows(); ihit++) {
            RecHit curHit = new RecHit();

            curHit.id = hitsBank.getShort("id", ihit) - 1;   // -1 for starting from 0
            curHit.cid = (short) (hitsBank.getShort("clusterId", ihit) - 1);  // -1 for starting from 0
            if (curHit.cid == -2 || !mchitsInECal.containsKey(curHit.id)) {
                continue; // The hit is not part of any cluster, or the hit it's corresponding MC hit is ignored
            }
            //System.out.println("Inside the hit loop:  the cid of the hit is " + curHit.cid);
            curHit.pindex = clId2Pindex.get(curHit.cid);
            curHit.detector = (byte) DetectorType.ECAL.getDetectorId();

            if (recHits.get(curHit.cid) == null) {
                recHits.put(curHit.cid, new ArrayList<>());
            }

            recHits.get(curHit.cid).add(curHit);
        }

        return recHits;
    }

    Map< Short, List<RecHit>> getFTCalHits(DataEvent event, Map<Integer, MCHit> mchitsInFTCal) {
        Map< Short, List<RecHit>> recHits = new HashMap<>();

        if (mchitsInFTCal == null) {
            /**
             * In case if no MC hit present in the FTCal, then don't proceed, as
             * we need only hits that are associated to an MC hit
             */
            return recHits;
        }

        /**
         * Check if two necessary banks exist otherwise will return null
         */
        if (event.hasBank("FTCAL::hits") == false) {
            return null;
        }

        if (event.hasBank("FTCAL::clusters") == false) {
            return null;
        }

        Map<Short, Short> clId2Pindex = new HashMap<>();
        DataBank RecFTBank = event.getBank("REC::ForwardTagger");

        for (int iFT = 0; iFT < RecFTBank.rows(); iFT++) {

            Short pindex = RecFTBank.getShort("pindex", iFT);
            Short index = RecFTBank.getShort("index", iFT);

            clId2Pindex.put(index, pindex);

            //System.out.println("Map clId2Pindex:   index = " + index + "pindex = " + pindex);
        }

        DataBank hitsBank = event.getBank("FTCAL::hits");

        for (int ihit = 0; ihit < hitsBank.rows(); ihit++) {
            RecHit curHit = new RecHit();

            curHit.id = hitsBank.getShort("hitID", ihit);   // Not removing 1, as hitID start from 0
            curHit.cid = (short) (hitsBank.getShort("clusterID", ihit) - 1);  // -1 for starting from 0
            if (curHit.cid == -2 || !mchitsInFTCal.containsKey(curHit.id)) {
                continue; // The hit is not part of any cluster, or the hit it's corresponding MC hit is ignored
            }
            //System.out.println("Inside the hit loop:  the cid of the hit is " + curHit.cid);

            /**
             * For FTCal not necessarily all clusters are associated to a rec
             * particle, that is why we will check, if the clId2Pindex contains
             * the given cluster
             */
            if (!clId2Pindex.containsKey(curHit.cid)) {
                continue;
            }

            curHit.pindex = clId2Pindex.get(curHit.cid);
            curHit.detector = (byte) DetectorType.ECAL.getDetectorId(); // Seems Wrong 10/03/2020, Should be looked at

            if (recHits.get(curHit.cid) == null) {
                recHits.put(curHit.cid, new ArrayList<>());
            }

            recHits.get(curHit.cid).add(curHit);
        }

        return recHits;
    }

    Map< Short, List<RecHit>> getCNDHits(DataEvent event, Map<Integer, MCHit> mchitsInCND) {
        Map< Short, List<RecHit>> recHits = new HashMap<>();

        if (mchitsInCND == null) {
            /**
             * If no MC hits present in the CND, then we stop here! no need to
             * collect hits, as wee need only hits that are matched to an MChit
             */
            return recHits;
        }

        /**
         * Check if three necessary banks exist otherwise will return null
         */
        if ((event.hasBank("CND::hits") == false) || (event.hasBank("CND::clusters") == false)
                || (event.hasBank("REC::Scintillator") == false)) {
            return null;
        }

        Map<Short, Short> clId2Pindex = new HashMap<>();
        DataBank RecScintil = event.getBank("REC::Scintillator");

        for (int iSC = 0; iSC < RecScintil.rows(); iSC++) {

            // Rec scintillator has different detectors in it, so we want only CND responces in this case
            if (RecScintil.getByte("detector", iSC) != (byte) DetectorType.CND.getDetectorId()) {
                continue;
            }

            Short pindex = RecScintil.getShort("pindex", iSC);
            Short index = RecScintil.getShort("index", iSC);

            clId2Pindex.put(index, pindex);

            //System.out.println("Map clId2Pindex:   index = " + index + "pindex = " + pindex);
        }

        DataBank hitsBank = event.getBank("CND::hits");

        for (int ihit = 0; ihit < hitsBank.rows(); ihit++) {
            RecHit curHit = new RecHit();

            curHit.id = hitsBank.getShort("id", ihit) - 1;   // -1, as id start from 1
            curHit.cid = (short) (hitsBank.getShort("clusterid", ihit) - 1);  // -1 for starting from 0
            if (curHit.cid == -2 || !mchitsInCND.containsKey(curHit.id)) {
                continue; // The hit is not part of any cluster, or the hit it's corresponding MC hit is ignored
            }
            //System.out.println("Inside the hit loop:  the cid of the hit is " + curHit.cid);

            curHit.pindex = clId2Pindex.get(curHit.cid);
            curHit.detector = (byte) DetectorType.CND.getDetectorId();

            if (recHits.get(curHit.cid) == null) {
                recHits.put(curHit.cid, new ArrayList<>());
            }

            recHits.get(curHit.cid).add(curHit);
        }

        return recHits;
    }

    Map< Short, List<RecHit>> getCTOFHits(DataEvent event, Map<Integer, MCHit> mchitsInCTOF) {
        Map< Short, List<RecHit>> recHits = new HashMap<>();

        if (mchitsInCTOF == null) {
            /**
             * If no MC hits present in the CTOF, then we stop here! no need to
             * collect hits, as wee need only hits that are matched to an MChit
             */
            return recHits;
        }

        /**
         * Check if three necessary banks exist otherwise will return null
         */
        if ((event.hasBank("CTOF::hits") == false) || (event.hasBank("CTOF::clusters") == false)
                || (event.hasBank("REC::Scintillator") == false)) {
            return null;
        }

        Map<Short, Short> clId2Pindex = new HashMap<>();
        DataBank RecScintil = event.getBank("REC::Scintillator");

        for (int iSC = 0; iSC < RecScintil.rows(); iSC++) {

            // Rec scintillator has different detectors in it, so we want only CTOF responces in this case
            if (RecScintil.getByte("detector", iSC) != (byte) DetectorType.CTOF.getDetectorId()) {
                continue;
            }

            Short pindex = RecScintil.getShort("pindex", iSC);
            Short index = RecScintil.getShort("index", iSC);

            clId2Pindex.put(index, pindex);
        }

        DataBank hitsBank = event.getBank("CTOF::hits");

        for (int ihit = 0; ihit < hitsBank.rows(); ihit++) {
            RecHit curHit = new RecHit();

            curHit.id = hitsBank.getShort("id", ihit) - 1;   // -1, as id starts from 1
            curHit.cid = (short) (hitsBank.getShort("clusterid", ihit) - 1);  // -1 for starting from 0
            if (curHit.cid == -1 || !mchitsInCTOF.containsKey(curHit.id)) {
                continue; // The hit is not part of any cluster, or the hit it's corresponding MC hit is ignored
            }
            //System.out.println("Inside the hit loop:  the cid of the hit is " + curHit.cid);

            curHit.pindex = clId2Pindex.get(curHit.cid);
            curHit.detector = (byte) DetectorType.CTOF.getDetectorId();

            if (recHits.get(curHit.cid) == null) {
                recHits.put(curHit.cid, new ArrayList<>());
            }

            recHits.get(curHit.cid).add(curHit);
        }

        return recHits;
    }

    List<RecCluster> getECalClusters(DataEvent event) {

        List<RecCluster> cls = new ArrayList<>();

        /**
         * We need the bank REC::Calorimeter, so as a first thing we will check
         * if the bank exist
         */
        if (event.hasBank("REC::Calorimeter") == false) {
            return cls;
        }

        DataBank recCal = event.getBank("REC::Calorimeter");

        //System.out.println("# of Cluster in the event is " + recCal.rows());
        for (int iCl = 0; iCl < recCal.rows(); iCl++) {
            RecCluster curCl = new RecCluster();

            curCl.id = recCal.getShort("index", iCl);
            curCl.pindex = recCal.getShort("pindex", iCl);
            curCl.detector = recCal.getByte("detector", iCl);
            curCl.layer = recCal.getByte("layer", iCl);
            curCl.sector = recCal.getByte("sector", iCl);
            curCl.energy = recCal.getFloat("energy", iCl);

            curCl.size = -1;    // For ECal clusters this is not particularly important, // We don't have this in the bank
            curCl.rectid = -1;  // We will not use ECal clusters for tracks.
            curCl.superlayer = -1; // Not applicable for ECal clusters

            cls.add(curCl);
        }

        return cls;
    }

    List<RecCluster> getFTCalClusters(DataEvent event) {
        List<RecCluster> cls = new ArrayList<>();

        /**
         * We need the bank REC::ForwardTagger, so as a first thing we will
         * check if the bank exist
         */
        if (event.hasBank("REC::ForwardTagger") == false) {
            return cls;
        }

        DataBank recFTCal = event.getBank("REC::ForwardTagger");

        for (int iCl = 0; iCl < recFTCal.rows(); iCl++) {

            /**
             * Both FT clusters and FT hodo hits are in the same
             * REC::ForwardTagger bank
             */
            if (recFTCal.getByte("detector", iCl) != DetectorType.FTCAL.getDetectorId()) {
                continue;
            }

            RecCluster curCl = new RecCluster();

            curCl.id = recFTCal.getShort("index", iCl);
            curCl.pindex = recFTCal.getShort("pindex", iCl);
            curCl.detector = recFTCal.getByte("detector", iCl);
            curCl.layer = recFTCal.getByte("layer", iCl);
            curCl.sector = -1; // No concept of sector for FT
            curCl.energy = recFTCal.getFloat("energy", iCl);
            curCl.size = recFTCal.getShort("size", iCl);

            curCl.rectid = -1;  // We will not use ECal clusters for tracks.
            curCl.superlayer = -1; // Not applicable for ECal clusters

            cls.add(curCl);
        }

        return cls;
    }

    List<RecCluster> getCNDClusters(DataEvent event) {
        List<RecCluster> cls = new ArrayList<>();

        /**
         * Of course we need the REC::Scintillator bank. Though we don't need
         * directly the CND::cluster bank, however without it REC::scintillator
         * will not have entry with CND detector
         */
        if ((event.hasBank("REC::Scintillator") == false) || (event.hasBank("CND::cluster") == false)) {
            return cls;
        }

        DataBank recSC = event.getBank("REC::Scintillator");

        for (int iSC = 0; iSC < recSC.rows(); iSC++) {

            if (recSC.getByte("detector", iSC) != DetectorType.CND.getDetectorId()) {
                continue;
            }

            RecCluster curCl = new RecCluster();
            curCl.id = recSC.getShort("index", iSC);
            curCl.pindex = recSC.getShort("pindex", iSC);
            curCl.detector = recSC.getByte("detector", iSC);
            curCl.layer = recSC.getByte("layer", iSC);
            curCl.sector = recSC.getByte("sector", iSC);
            curCl.energy = recSC.getFloat("energy", iSC);
            curCl.size = -1; // For CND clusters this is not a relevant variable
            curCl.rectid = -1; // CND is not used for tracks
            curCl.superlayer = -1; // not applicable

            cls.add(curCl);
        }

        return cls;
    }

    List<RecCluster> getCTOFClusters(DataEvent event) {
        List<RecCluster> cls = new ArrayList<>();

        /**
         * Of course we need the REC::Scintillator bank. Though we don't need
         * directly the CTOF::cluster bank, however without it REC::scintillator
         * will not have entry with CTOF detector
         */
        if ((event.hasBank("REC::Scintillator") == false) || (event.hasBank("CTOF::cluster") == false)) {
            return cls;
        }

        DataBank recSC = event.getBank("REC::Scintillator");

        for (int iSC = 0; iSC < recSC.rows(); iSC++) {

            if (recSC.getByte("detector", iSC) != DetectorType.CTOF.getDetectorId()) {
                continue;
            }

            RecCluster curCl = new RecCluster();
            curCl.id = recSC.getShort("index", iSC);
            curCl.pindex = recSC.getShort("pindex", iSC);
            curCl.detector = recSC.getByte("detector", iSC);
            curCl.layer = recSC.getByte("layer", iSC);
            curCl.sector = recSC.getByte("sector", iSC);
            curCl.energy = recSC.getFloat("energy", iSC);
            curCl.size = -1; // For CTOF clusters this is not a relevant variable
            curCl.rectid = -1; // CTOF is not used for tracks
            curCl.superlayer = -1; // not applicable

            cls.add(curCl);
        }

        return cls;
    }

    /**
     *
     * @param cls: List of clusters for a given detector
     * @param Rechits_a : Map<clId, ListRecHis>, i.e. list of hits for each
     * cluster
     * @param mchits : Map<hitn, mchit>, map of mc hits, where the Key is the
     * "hitn"
     */
    void MatchClasters(List<RecCluster> cls, Map< Short, List<RecHit>> Rechits_a, Map<Integer, MCHit> mchits) {

        if (cls == null) {
            return;
        }

        for (RecCluster cl : cls) {

            /**
             * The Key of the map is the MCParticleindex, that created a given
             * hit in a cluster, while the value of the map shows # of hits in
             * the cluster from that MCParticle
             */
            Map<Integer, Integer> matchMCParts = new HashMap<>();

            List<RecHit> recHits = Rechits_a.get(cl.id);

            if (recHits == null) {
                /**
                 * ************* AA TT EE NN TT II OO NN *******************
                 * This needs to be resolved!! There should not be cases where
                 * there is a cluster, but non of hits has clusterId pointing to
                 * that cluster. A possibility is that because of shared hits
                 * all hits of a given cluster is shared wit other cluster(s).
                 *
                 * Needs to be investigated
                 */
//                System.out.println("Oo, recHits is Null. Tot # of cluster is " + cls.size() + "    The energy of the cluster is " + cl.energy);
//                System.out.println("Cluster is " + cl.toString());
                continue;
                //System.out.println(" ====== # of hit in a cluster " + cl.id + " is " + recHits.size() + " =========");
            }

            for (RecHit curRecHit : recHits) {

                MCHit mchit = mchits.get(curRecHit.id);

                if (mchit != null) {
                    //cl.nHitMatched = (short) (cl.nHitMatched + (short) 1);

                    if (matchMCParts.get(mchit.otid) == null) {
                        matchMCParts.put(mchit.otid, 1);
                    } else {
                        matchMCParts.put(mchit.otid, matchMCParts.get(mchit.otid) + 1);
                    }
                } else {
                    //System.err.println("******* No MC hit is matched to this RecHit");

//                    for (int hitn : mchits.keySet()) {
//                        System.err.println("hitn is " + hitn + " otid is " + mchits.get(hitn).otid);
//                    }
//                    System.err.println(curRecHit.toString());
                }

            }

            /**
             * Loop over all hits is finished, so let's check, if there are more
             * than one MCParticle is associated with the cluster, chose the one
             * which has largest # of hits.
             */
            Map.Entry<Integer, Integer> maxEntry = null;
            for (Map.Entry<Integer, Integer> entry : matchMCParts.entrySet()) {
                if (maxEntry == null || entry.getValue().compareTo(maxEntry.getValue()) > 0) {
                    maxEntry = entry;
                }
            }

            if (maxEntry != null) {
                cl.nHitMatched = maxEntry.getValue();
                cl.mcotid = maxEntry.getKey().shortValue();
                //System.out.println("Final otid is  " + maxEntry.getKey() + "and # of matched hits is " + maxEntry.getValue());
            }

        }

    }

    /**
     *
     * @param mcpKeys: Set of MCparticle iDs,
     * @param cls : List of clusters
     * @return : Map<MCPId, List<Clusters>>, returns list of RecClusters for
     * each MC particle
     */
    Map<Short, List<RecCluster>> mapClustersToMCParticles(Set<Short> mcpKeys, List<RecCluster> cls) {
        Map<Short, List<RecCluster>> map = new HashMap<>();

        for (short theKey : mcpKeys) {

            map.put(theKey, new ArrayList<>());
        }

        //System.out.println(" ** ** ** Size of the cls is " + cls.size());
        for (RecCluster curCl : cls) {

            if (map.get(curCl.mcotid) == null) {
                // Should not happen, but just in case
                map.put(curCl.mcotid, new ArrayList<>());
            }

            map.get(curCl.mcotid).add(curCl);
        }

        //System.out.println(" ** ** ** Size of the Map is " + map.size());
        return map;
    }

    List<MCRecMatch> MakeMCRecMatch(Map<Short, MCPart> mcp, Map<Short, List<RecCluster>> clsPerMCp) {

        List<MCRecMatch> recMatch = new ArrayList<>();

        for (short imc : mcp.keySet()) {

            MCRecMatch match = new MCRecMatch();

            match.id = imc;

            /**
             * Generally speaking it is possible that all clusters of a given MC
             * particles will not have the same Rec::Particle. So we will make a
             * map here Map<pindex, count>, and the matched Rec::Particle will
             * be the one with highest count.
             */
            Map<Short, Integer> matched_counts = new HashMap<>();

            Map<Short, Integer> matched_PCalcounts = new HashMap<>();
            Map<Short, Integer> matched_ECcounts = new HashMap<>();
            Map<Short, Integer> matched_FTCalcounts = new HashMap<>();
            Map<Short, Integer> matched_CNDcounts = new HashMap<>();
            Map<Short, Integer> matched_CTOFcounts = new HashMap<>();
            Map<Short, Integer> matched_BSTcounts = new HashMap<>();
            Map<Short, Integer> matched_BMTcounts = new HashMap<>();

            int nPCal = 0;
            int nEC = 0;
            int nFTCal = 0;
            int nCND = 0;
            int nCTOF = 0;
            int nBST = 0;
            int nBMT = 0;

            /**
             * Making sure there are clusters created from the given MCParticle
             */
            if (!clsPerMCp.get(imc).isEmpty()) {

                match.nclusters = (short) clsPerMCp.get(imc).size();

                //System.out.println("# of clusters is " + match.nclusters);
                for (RecCluster curCl : clsPerMCp.get(imc)) {

                    incrementMap(matched_counts, curCl.pindex);
                    //System.out.println("******* Counts after Increment Operation is " + matched_counts.get(curCl.pindex));

                    final int det = (int) curCl.detector;

                    /**
                     * Can not use swith with with det.getdetectorID() so will
                     * make
                     */
                    switch (det) {

                        case ECALID:

                            if (curCl.layer == 1) {
                                // PCal
                                incrementMap(matched_PCalcounts, curCl.pindex);
                            } else if (curCl.layer == 4 || curCl.layer == 7) {
                                // EC
                                incrementMap(matched_ECcounts, curCl.pindex);
                            }
                            break;
                        case FTCALID:
                            incrementMap(matched_FTCalcounts, curCl.pindex);

                        case BMTID:
                            incrementMap(matched_BMTcounts, curCl.pindex);
                            break;
                        case BSTID:
                            incrementMap(matched_BSTcounts, curCl.pindex);
                            break;
                        case CNDID:
                            incrementMap(matched_CNDcounts, curCl.pindex);
                        case CTOFID:
                            incrementMap(matched_CTOFcounts, curCl.pindex);
                            break;
                    }

                }

                if (mcp.get(match.id).pid == PHOTON_ID || mcp.get(match.id).pid == NEUTRON_ID) {
                    match.pindex = getMaxEntryKey(matched_counts);
                    match.tid = -1;
                    match.nClsInRec = matched_counts.get(match.pindex).shortValue();
                    match.frac = (float) (match.nClsInRec / match.nclusters);
                    //if (0 < matched_PCalcounts.get(match.pindex) || 0 < matched_ECcounts.get(match.pindex)) {
                    if ((matched_PCalcounts.containsKey(match.pindex) && matched_PCalcounts.get(match.pindex) > 0)
                            || (matched_ECcounts.containsKey(match.pindex) && 0 < matched_ECcounts.get(match.pindex))
                            || (matched_FTCalcounts.containsKey(match.pindex) && matched_FTCalcounts.get(match.pindex) > 0)
                            || ((matched_CNDcounts.containsKey(match.pindex) && matched_CNDcounts.get(match.pindex) > 0)
                            ||  (matched_CTOFcounts.containsKey(match.pindex) && matched_CTOFcounts.get(match.pindex) > 0))) {
                        match.reconstructable = 1;
                    } else {
                        match.reconstructable = 0;
                    }
                }

            } else {
                match.pindex = -1;
                match.tid = -1;
                match.nClsInRec = 0;
                match.frac = 0;
                match.reconstructable = 0;
            }

            recMatch.add(match);
        }

        return recMatch;
    }

    void bankWriter(DataEvent event, List<MCRecMatch> mcp, List<RecCluster> cls) {

        DataBank bank = event.createBank("MC::IsParticleMatched", mcp.size());

        for (int j = 0; j < mcp.size(); j++) {
            MCRecMatch p = mcp.get(j);
            bank.setShort("mcTindex", j, p.id);
            bank.setShort("recTindex", j, p.tid);
            bank.setShort("pindex", j, p.pindex);
            bank.setShort("nMCclusters", j, (short) p.nclusters);
            bank.setByte("isInAcc", j, p.reconstructable);
            bank.setFloat("fraction", j, p.frac);
        }

        event.appendBanks(bank);
    }

    /**
     * Some Utility functions
     */
    void PrintRecHits(Map< Short, List<RecHit>> Rechits_a) {

        System.out.println("******************* Map of Rec Hits ******************************");

        for (Map.Entry<Short, List<RecHit>> entry : Rechits_a.entrySet()) {
            System.out.println("**  ======= The key (clusterID) : is " + entry.getKey() + " ======= ");

            for (int ilist = 0; ilist < entry.getValue().size(); ilist++) {

                System.out.println("**  *id* of the " + ilist + "-th RecHit is " + entry.getValue().get(ilist).id);
                System.out.println("**  *pindex of the " + ilist + "-th RecHit is " + entry.getValue().get(ilist).pindex);
                System.out.println("**  *cid* of the " + ilist + "-th RecHit is " + entry.getValue().get(ilist).cid);
                System.out.println("**  *detector* of the " + ilist + "-th RecHit is " + entry.getValue().get(ilist).detector);
                System.out.println("**");
            }
        }

        System.out.println("******************* Map of Rec Hits ******************************");

    }

    void PrintRecCluster(RecCluster cl) {

        System.out.println("******************* RecCluster ******************************");
        System.out.println("** id is                " + cl.id);
        System.out.println("** mctid is             " + cl.mcotid);
        System.out.println("** rectid is            " + cl.rectid);
        System.out.println("** pindex is            " + cl.pindex);
        System.out.println("** nHittMatched is      " + cl.nHitMatched);
        System.out.println("** size is              " + cl.size);
        System.out.println("** detector is          " + cl.detector);
        System.out.println("** layer is             " + cl.layer);
        System.out.println("** superlayer is        " + cl.superlayer);
        System.out.println("** sector is            " + cl.sector);
        System.out.println("******************* RecCluster ******************************");
    }

    void PrintRecMatches(List<MCRecMatch> matches) {

        System.out.println("** ************  MATCHED Objects *********************");
        for (int i = 0; i < matches.size(); i++) {

            String strIndex;
            if (i == 0) {
                strIndex = "1st";
            } else if (i == 1) {
                strIndex = "2nd";
            } else if (i == 2) {
                strIndex = "3rd";
            } else {
                strIndex = String.valueOf(i + 1) + "th";
            }

            System.out.println("** ********" + strIndex + "Particle *********");
            System.out.println("** id = " + matches.get(i).id);
            System.out.println("** pindex = " + matches.get(i).pindex);
            System.out.println("** nclusters = " + matches.get(i).nclusters);
            System.out.println("** nclustersInRec = " + matches.get(i).nClsInRec);
            System.out.println("** frac = " + matches.get(i).frac);
            System.out.println("** reconstructable = " + matches.get(i).reconstructable);
            System.out.println("");

        }
        System.out.println("** *********  End of MATCHED Objects *****************");

    }

    void PrintClsPerMc(Map<Short, List<RecCluster>> map) {

        System.out.println("** ******** Map of Clusters per MC particle **************");

        if (!map.isEmpty()) {

            for (Short curKey : map.keySet()) {

                int nCl = map.get(curKey).size();
                System.out.println("mcotid  = " + curKey + "     # of clusters is " + nCl);

                for (int ii = 0; ii < nCl; ii++) {

                    System.out.println("                ***** printing the cluster #" + ii + " *****");
                    System.out.print(map.get(curKey).get(ii).toString());
                }
            }

        } else {
            System.out.println("The Map is empty");
        }

        System.out.println("** ***** End of Map of Clusters per MC particle **********");

    }

    /**
     * This function as an argument expects counter maps, i.e. maps, which have
     * the value as a counter of Keys. I will increment the value of the map
     * each time it is called, and if the key doesn't exist, it will make an
     * entry with that key and will assign value = 1
     *
     * @param <T> : the type of the make Key
     * @param map : the map
     * @param var : is the key of the map
     */
    private <T> void incrementMap(Map<T, Integer> map, T var) {
        if (map.get(var) == null) {
            map.put(var, 1);
        } else {
            map.put(var, map.get(var) + 1);
        }
    }

    /**
     * Returns the Key of the map that has the highest counts
     *
     * @param <T> Type of the Map keys
     * @param map The actuall map
     * @return The Key, which has highest counts
     */
    private <T> T getMaxEntryKey(Map<T, Integer> map) {

        if (map.isEmpty()) {
            System.out.println("Oho Map is empty. Returning null");
            return null;
        }

        Map.Entry<T, Integer> maxEntry = null;
        for (Map.Entry<T, Integer> entry : map.entrySet()) {
            if (maxEntry == null || entry.getValue().compareTo(maxEntry.getValue()) > 0) {
                maxEntry = entry;
            }
        }

        return maxEntry.getKey();
    }
}
