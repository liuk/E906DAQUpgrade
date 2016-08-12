#include <iostream>

#include <TList.h>
#include <TH1.h>
#include <TF1.h>
#include <TString.h>
#include <TH1D.h>
#include <TROOT.h>
#include <TSystem.h>
#include <TFile.h>
#include <TTree.h>
#include <stdio.h>
#include <stdlib.h>

#include "THaCodaFile.h"
#include "THaEtClient.h"

using namespace std;

#define rocMask    0x00ff0000
#define boardMask  0xffff0000
#define eventMask  0x0000ffff
#define nWordsMask 0x7fff0000

const int NROC = 15;

Double_t NonCaliFineP[9] = {0.222, 0.666, 1.110, 1.554, 1.998, 2.442, 2.886, 3.330, 3.776};
Double_t NonCaliFineN[9] = {0,     0.145, 0.555, 1.085, 1.615, 2.145, 2.675, 3.205, 3.735};

double GetTDC(int Trig, int Data, int Postive, int& FineTrigBin, int& FineDataBin);
void InitArray();

int main(int argc, char* argv[])
{
    THaCodaData* coda;      // THaCodaData is abstract

    coda = new THaCodaFile(TString(argv[1]));
    TFile* saveFile = new TFile(argv[2], "recreate");
    TTree* saveTree = new TTree("save", "save");

    int runID;
    int eventID;
    int codaEventID;
    int codaEventType;
    int triggerType;
    int vxTicks;

    int rocID;
    int boardID;
    int channelID;
    int hitID;
    int posEdge;
    double tdcTime;

    int FineTrigBin;
    int FineDataBin;
    double FineTrig;
    double FineData;

    int tdcCSR;
    int tdcHeader;

    saveTree->Branch("runID",         &runID, "runID/I");
    saveTree->Branch("eventID",       &eventID, "eventID/I");
    saveTree->Branch("codaEventID",   &codaEventID, "codaEventID/I");
    saveTree->Branch("codaEventType", &codaEventType, "codaEventType/I");
    saveTree->Branch("triggerType",   &triggerType, "triggerType/I");
    saveTree->Branch("vxTicks",       &vxTicks, "vxTicks/I");

    saveTree->Branch("rocID",         &rocID, "rocID/I");
    saveTree->Branch("boardID",       &boardID, "boardID/I");
    saveTree->Branch("channelID",     &channelID, "channelID/I");
    saveTree->Branch("hitID",         &hitID, "hitID/I");
    saveTree->Branch("posEdge",       &posEdge, "posEdge/I");
    saveTree->Branch("tdcTime",       &tdcTime, "tdcTime/D");
    
    saveTree->Branch("FineDataBin",   &FineDataBin, "FineDataBin/I");
    saveTree->Branch("FineTrigBin",   &FineTrigBin, "FineTrigBin/I");
    saveTree->Branch("FineData",      &FineData, "FineData/D");
    saveTree->Branch("FineTrig",      &FineTrig, "FineTrig/D");

    saveTree->Branch("tdcCSR",        &tdcCSR, "tdcCSR/I");
    saveTree->Branch("tdcHeader",     &tdcHeader, "tdcHeader/I");

    int* data;
    codaEventID = 1;
    while(true)
    {
        int status = coda->codaRead();
        if(status != 0)
        {
            if(status == -1)
            {
                coda->codaClose();
                break;
            }
            else
            {
                cout << "ERROR: codaread status = " << hex << status << endl;
                continue;
            }
        }

        data = coda->getEvBuffer();
        codaEventType = data[1] >> 16;
        if(codaEventType == 0x14) 
        {
            cout << "Normal END, codaEventID = " << codaEventID << ", exiting ..." << endl;
            break;
        }

        if(codaEventID == 1)
        {
            runID = data[3];
            cout << "Prestart event for run " << runID << endl;

            ++codaEventID;
            continue;
        }
        else if(codaEventID == 2 || codaEventID == 3)   //user input and Go event
        {
            ++codaEventID;
            continue;
        }
        else if(codaEventID == 4)
        {
            cout << "Header event ... " << endl;

            tdcCSR = data[3] & 0xffff;
            printf("- FEE information: TDCcsr = %x\n", tdcCSR);

            ++codaEventID;
            continue;
        }
        else if(codaEventID == 5)
        {
            cout << "First event ..." << endl;
        }

        int nWordsTotal = data[0] + 1;
        eventID = data[4];

        int iWord = 7;
        hitID = 0;
        while(iWord < nWordsTotal) //entry once every ROC
        {
            cout << iWord << "  " << nWordsTotal << endl;

            //Read total words in this ROC
            int nWordsRoc = data[iWord++];
            int maxWordID = iWord + nWordsRoc; //actually the max wordID + 1

            //Read ROC id 
            rocID = (data[iWord++] & rocMask) >> 16;

            //Skip next two words
            ++iWord; ++iWord;

            //Read vme ticks
            vxTicks = data[iWord++];

            //Read each TDC
            while(iWord < maxWordID) //entry once every TDC
            {
                if(data[iWord] == 0xe906f00f)
                {
                    triggerType = data[iWord++];
                }
                else if(data[iWord] == 0xe906f010)  //begin of TDC
                {
                    ++iWord;
                    boardID = (data[iWord++] & boardMask) >> 16;

                    if(data[iWord] == 0xe906c0da)  //unexpected error
                    { 
                        cout << "Error in this event!" << endl;
                        ++iWord;
                    }
                    
                    if(data[iWord] == 0xe906e906)   //Normal end
                    {
                        ++iWord;
                    }
                    
                    if(((data[iWord] >> 31) & 1) == 1) //header
                    {
                        tdcHeader = data[iWord++];
                        
                        int nWordsTDC = (tdcHeader & 0x7ff00000) >> 20;
                        cout << iWord << "  " << nWordsTDC << endl;
                        for(int j = 0; j < nWordsTDC; ++j, ++iWord)
                        {
                            channelID = (((data[iWord] >> 28) & 0x3) << 4) + ((data[iWord] >> 24) & 0xf);
                            posEdge = (data[iWord] & 0x10000) >> 16;
                            tdcTime = GetTDC(tdcHeader & 0xffff, data[iWord] & 0xffff, posEdge, FineTrigBin, FineDataBin);

                            ++hitID;
                            saveTree->Fill();
                        }
                    }
                }
                else
                {
                    ++iWord;
                }
            }
        }

        ++codaEventID;
    }

    coda->codaClose();

    saveFile->cd();
    saveTree->Write();
    saveFile->Close();

    return 0;
}

double GetTDC(int Trig, int Data, int Positive, int& FineTrigBin, int& FineDataBin)
{
    int Trig4ns, Data4ns, TDC4ns;
    Trig4ns = (Trig & 0xfff0) >> 4;
    Data4ns = (Data & 0xfff0) >> 4;
    TDC4ns = (Trig4ns - Data4ns) & 0x1ff;
    if(Trig4ns < Data4ns) Trig4ns += 0x400;

    int TrigFine, DataFine;
    TrigFine = (Trig & 0xf) - 1;
    DataFine = Positive == 1 ? (Data & 0xf) - 1 : (Data & 0xf);

    FineTrigBin = (Trig & 0xf) - 1;
    FineDataBin = Data & 0xf;

    double TDC;
    if(Positive == 1) //postive edge
    {
        TDC = (Trig4ns*4 + (4. - NonCaliFineP[TrigFine])) - (Data4ns*4 + (4 - NonCaliFineP[DataFine]));//leading edge
    }
    else
    {
        TDC = (Trig4ns*4 + (4. - NonCaliFineP[TrigFine])) - (Data4ns*4 + NonCaliFineN[DataFine]);      //falling edge
    }

    return TDC;
}
