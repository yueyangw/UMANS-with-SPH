/* UMANS: Unified Microscopic Agent Navigation Simulator
** MIT License
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettr�
**
** Permission is hereby granted, free of charge, to any person obtaining
** a copy of this software and associated documentation files (the
** "Software"), to deal in the Software without restriction, including
** without limitation the rights to use, copy, modify, merge, publish,
** distribute, sublicense, and/or sell copies of the Software, and to
** permit persons to whom the Software is furnished to do so, subject
** to the following conditions:
**
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
** NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
** LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
** ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
** Contact: crowd_group@inria.fr
** Website: https://project.inria.fr/crowdscience/
** See the file AUTHORS.md for a list of all contributors.
*/

#include <tools/localsearch.h>
#include <iostream>

using namespace std;

double LocalSearch::backtr(const Vector2D &x0, const Vector2D& grad, const std::function<float(const Vector2D&)>& F, double alpha_0, double gamma, double delta, double rhok)
{
    /*
        https://www.mathworks.com/matlabcentral/fileexchange/45572-backtracking-armijo-type?s_tid=FX_rc2_behav
        %% BACKTRACKING ARMIJO-TYPE
        % DESCRIPTION:
        % Search method along a coordinate axis in which the search should be conducted
        % in both directions of the axis. It should also take into account the fact that
        % one direction dk can be assigned such that alpha=0 represents a
        % local minimum point of the function g(alpha)=F(xk+alpha*dk), for which
        % may not be able to find positive or negative values ??of alpha
        % close to 0 for which g(alpha)<g(0). If you do not want to use any
        % derivative, numerical "finished" procedures must define can
        % discriminate the situation. The model presented is an outline
        % Backtracking Armijo-type, based on the condition of acceptability of type "Parabolic".
        %
        % function [alpha] = backtr(alpha_guess,Xk,dk,F,gamma,delta,rhok)
        % INPUT:
        %       NOTE: (*) indicates necessary input, the other variables are optional
        %       (*) alpha _guess - current steplength (1*1) [>0];
        %       (*) Xk           - current iterate    (N*1);
        %       (*) dk           - search direction   (N*1);
        %           gamma        - constant provided by the user (1*1) [>0];
        %           delta        - constant provided by the user (1*1) into the range [0,  1];
        %           rhok         - constant provided by the user (1*1) into the range [0,  1];
        %       (*) F            - function handle of the objective function (RN->R );
        % OUTPUT:
        %       alpha - value of alpha whether the condition holds (1*1);
        % REVISION:
        %       Ennio Condoleo - 21.15 13 Feb 2014
        % REFERENCE: https://goo.gl/TUoCYu
        %
    */

    float Fx0 = F(x0);

	float dkSqrMag = grad.sqrMagnitude();
	float dkMag = sqrtf(dkSqrMag);

    // positive direction (+)alpha
    double alpha = alpha_0;
    while (F(x0+(float)alpha*grad)>Fx0-gamma*alpha*alpha*dkSqrMag)
    {
        if (alpha*dkMag < rhok)
            alpha = 0;             //  % <-- failure to search for a value of alpha nonzero
        else
            alpha = alpha*delta;   //  % <-- reduction of the steplength
    }

    double alpha1 = alpha;
    double F1 = F(x0+(float)alpha1*grad)-(Fx0-gamma*alpha1*alpha1*dkSqrMag);

    // negative direction (-)alpha
    alpha = alpha_0;
    while (F(x0-(float)alpha*grad)>Fx0-gamma*alpha*alpha*dkSqrMag)
    {
        if (alpha*dkMag < rhok)
            alpha = 0;            //    % <-- failure to search for a value of alpha nonzero
        else
            alpha = alpha*delta;  //    % <-- reduction of the steplength
    }

    double alpha2 = -alpha;
    double F2 = F(x0+(float)alpha2*grad)-(Fx0-gamma*alpha2*alpha2*dkSqrMag);

    // choice of the value of alpha for which it is provided with sufficient reduction
	return (F1 < F2 ? alpha1 : alpha2);
}
