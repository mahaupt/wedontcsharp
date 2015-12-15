#include "mex.h"
#include <vector>
#include <cmath>

//Betrag eines 2D-Vektors
double norm(double x, double y)
{
    return sqrt((x * x) + (y * y));
}

//Auf Banden- und Minentreffer testen
bool HitTest(double pos[],double r,const mxArray * minen)
{
    //Bandentreffer
    if (pos[0] - r <= 0 || pos[1] - r <= 0 || pos[0] + r >= 1 || pos[1] + r >= 1)
    {
        return true;
    }
    
    
    //Strukturlänge überprüfen
    int l = (int)mxGetNumberOfElements(minen);
    //Variablen deklarieren
    double rM;
    double* pM;
        
    //Alle Minen testen
    for (int i = 0;i < l;i++)
    {
        //Radius abrufen
        rM = mxGetScalar(mxGetField(minen,i,"radius"));
        //Position abrufen
        pM = mxGetPr(mxGetField(minen,i,"pos"));
        
        //Kreise schneiden sich
        if (pow(r + rM,2) >= (pow(pM[0] - pos[0],2) + pow(pM[1] - pos[1],2)))
        {
            return true;
        }
    }
    
    
    //Kein Treffer
    return false;
}

//Berechnungsalgorithmus
std::vector<std::vector<double>> wegPunkte(double pos[], double ges[], double t_end, double grad[], int gradL, double bes, double dt, double radius, const mxArray *minen)
{
    //Rückgabewerte
    std::vector<double> X, Y, G;
    
    //Bewegen wir uns überhaupt?
    if (norm(ges[0],ges[1]) != 0)
    {
        //Indizes initialisieren
        int index = 0;
        int st_index = 0;
        
        //Umformungskonstate
        const double deg2rad = acos(-1.0)/180;
        
        for (int i = 0;i < gradL;i++)
        {
            //Temporäre Variablen initialisieren
            double t_pos[] = {pos[0], pos[1]};
            double t_ges[] = {ges[0], ges[1]};
            double t_grad = grad[i];
            double t_rad = t_grad * deg2rad;
            double t_sin = sin(t_rad);
            double t_cos = cos(t_rad);
            
            //Zwischenindex
            st_index = index;
            
            for (double t = 0;t < t_end;t += dt)
            {
                //Geschwindigkeitsbetrag ermitteln
                double n_ges = norm(t_ges[0],t_ges[1]);
                
                //Beschleunigung ermitteln
                double t_besX = (t_ges[0] * t_cos + t_ges[1] * t_sin) / n_ges * bes;
                double t_besY = (t_ges[1] * t_cos - t_ges[0] * t_sin) / n_ges * bes;
                
                //Geschwindigkeit ermitteln
                t_ges[0] = t_ges[0] + t_besX * dt;
                t_ges[1] = t_ges[1] + t_besY * dt;
                
                //Position ermitteln
                t_pos[0] = t_pos[0] + t_ges[0] * dt;
                t_pos[1] = t_pos[1] + t_ges[1] * dt;
                
                
                //HitTest durchführen
                if (HitTest(t_pos, radius, minen))
                {   
                    //Ungültig, also ganze Kreisbahn löschen
                    X.erase(X.begin() + st_index, X.end());
                    Y.erase(Y.begin() + st_index, Y.end());
                    G.erase(G.begin() + st_index, G.end());
                    index = st_index;
                    break;
                }
                else
                {   
                    //Bisher alles gut
                    X.push_back(t_pos[0]);
                    Y.push_back(t_pos[1]);
                    G.push_back(t_grad);
                    index++;
                }
            }
        }
    }
    else
    {
        X.push_back(pos[0]);
        Y.push_back(pos[1]);
        G.push_back(0);
    }
    
    return std::vector<std::vector<double>> {X, Y, G};
}

//Einstiegspunkt
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //Anzahl der Eingangsparamter überprüfen
    if(nrhs != 8)
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Acht Eingabevariablen benötigt.");
    }
    
    //Anzahl der Ausgangsparamter überprüfen
    if(nlhs != 3)
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Ouput","Drei Ausgabevariablen benötigt.");
    }
    
    //Ersten Eingabeparameter (Position) prüfen
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 2)
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Die Position muss ein reeller 2D-Vektor sein.");
    }
    
    //Zweiten Eingabeparameter (Geschwindigkeit) prüfen
    if (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 2)
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Die Geschwindigkeit muss ein reeller 2D-Vektor sein.");
    }
    
    //Dritten Eingabeparameter (Endzeit) prüfen
    if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]))
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Die Endzeit muss eine reelle Zahl sein.");
    }
    
    //Vierten Eingabeparameter (Gradraster) prüfen
    if (!mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]))
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Das Gradraster muss ein reeller Vektor sein.");
    }
    
    //Fünften Eingabeparameter (Betrag der Beschleunigung) prüfen
    if (!mxIsDouble(prhs[4]) || mxIsComplex(prhs[4]))
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Der Beschleunigungsbetrag muss eine reelle Zahl sein.");
    }
    
    //Sechsten Eingabeparameter (Zeitintervall) prüfen
    if (!mxIsDouble(prhs[5]) || mxIsComplex(prhs[5]))
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Das Zeitintervall muss eine reelle Zahl sein.");
    }
    
    //Siebten Eingabeparameter (Radius + Margin) prüfen
    if (!mxIsDouble(prhs[6]) || mxIsComplex(prhs[6]))
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Der Radius (inklusive Sicherheitsmargin) muss eine reelle Zahl sein.");
    }
    
    //Achten Eingabeparameter (Minenstruktur) prüfen
    if (!mxIsStruct(prhs[7]))
    {
        mexErrMsgIdAndTxt("The_sBaseballs:beschleunigung:Input","Das achte Eingangsargument muss eine Struktur sein.");
    }
    
    
    //Funktion aufrufen
    std::vector<std::vector<double>> K = wegPunkte(mxGetPr(prhs[0]), mxGetPr(prhs[1]), mxGetScalar(prhs[2]), mxGetPr(prhs[3]), mxGetNumberOfElements(prhs[3]), mxGetScalar(prhs[4]), mxGetScalar(prhs[5]), mxGetScalar(prhs[6]), prhs[7]);
    
    //Vektoren abrufen
    std::vector<double> X = K.at(0);
    std::vector<double> Y = K.at(1);
    std::vector<double> G = K.at(2);
    
    //Rückgabewerte zuweisen
    plhs[0] = mxCreateDoubleMatrix(1,(int)X.size(),mxREAL);
    std::copy(X.begin(),X.end(),mxGetPr(plhs[0]));
    
    plhs[1] = mxCreateDoubleMatrix(1,(int)Y.size(),mxREAL);
    std::copy(Y.begin(),Y.end(),mxGetPr(plhs[1]));
    
    plhs[2] = mxCreateDoubleMatrix(1,(int)G.size(),mxREAL);
    std::copy(G.begin(),G.end(),mxGetPr(plhs[2]));
}