function qDotDot=feedbackLinear(Ftelda,Bq,Cq,Gq)
    qDotDot=inv(Bq)*(-Cq-Gq)+Ftelda;
end