#pragma once

#include <QObject>
#include <memory>
#include <vector>
#include <string>

#include "imageprovider.h"
#include "faceparts.h"
#include "pupilfinder.h"
#include "gazehyps.h"
#include "abstractlearner.h"

Q_DECLARE_METATYPE(std::string) // deklarira klasu std::string kao meta type. Sad QMetaType koji se bavi tipovima u meta-object
                                // sustavu zna za tip std::string.

// Razred (class) je samo poseban tip podataka. Potrebna je deklaracija koja se sastoji od zaglavlja i tijela. Zaglavlje se
// sastoji od kljucne rijeci class i naziva razreda (identifikatora koji mora biti jedinstven).
class WorkerThread : public QObject
{
    Q_OBJECT // https://en.wikipedia.org/wiki/Meta-object_System

// definiranje prava pristupa - private, public i protected. Ako se pravo pristupa ne navede eksplicitno, za razrede se
// podrazumijeva privatni pristup
private:
    bool shouldStop = false;
    // http://stackoverflow.com/questions/6876751/differences-between-unique-ptr-and-shared-ptr
    std::unique_ptr<ImageProvider> getImageProvider();
    void normalizeMat(const cv::Mat &in, cv::Mat &out);
    void dumpPpm(std::ofstream &fout, const cv::Mat &frame);
    void dumpEst(std::ofstream &fout, GazeHypsPtr gazehyps);
    void writeEstHeader(std::ofstream& fout);
    void interpretHyp(GazeHyp &ghyp);
    void smoothHyp(GazeHyp& ghyp);

// takozvano javno sucelje programa
public:
    explicit WorkerThread(QObject *parent = 0); // kada se stvori objekt nekog razreda, članovi su neinicijalizirani
                                                // stoga se koristi konstrukcijski funkcijski član zvan konstruktor.
                                                // Osnovna zadaca konstruktora jest inicijalizacija objekta tako da se dodijeli
                                                // određeni memorijski prostor. Konstruktor ne alocira memoriju za pohranjivanje
                                                // članova objekata, no može dodatno alocirati memoriju koju objekt koristi.

    // explicit znaci da nema konverzije argumenata, kao sto bi compiler mogao napraviti u slucaju da se za argument ne posalje
    // tip argumenta koji je definiran za konstruktor. kad se kaze explicit, onda mora biti dan argument koji je predviden.
    int threadcount = 6;
    int desiredFps = 0;
    cv::Size inputSize;

    // http://www.cplusplus.com/reference/string/string/ -> dokumentacija std::string razreda
    std::string inputType;
    std::string inputParam;
    std::string modelfile;
    std::string classifyGaze;
    std::string trainGaze;
    std::string classifyLid;
    std::string trainLid;
    std::string streamppm;
    std::string trainGazeEstimator;
    std::string trainLidEstimator;
    std::string trainVerticalGazeEstimator;
    std::string estimateGaze;
    std::string estimateVerticalGaze;
    std::string estimateLid;
    std::string dumpEstimates;

    double limitFps = 0;
    double horizGazeTolerance = 5;
    double verticalGazeTolerance = 5;
    bool smoothingEnabled = false;
    bool showstats = true;
    TrainingParameters trainingParameters;

// http://doc.qt.io/qt-4.8/signalsandslots.html

signals:
    void finished();
    void imageProcessed(GazeHypsPtr gazehyps);
    void statusmsg(std::string msg);

public slots:
    void process();
    void stop();
    void setHorizGazeTolerance(double tol);
    void setVerticalGazeTolerance(double tol);
    void setSmoothing(bool enabled);
}; // deklaracija razreda uvijek zavrsava znakom ; -> pazi na to!
