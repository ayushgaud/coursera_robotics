function [ coeff ] = diffcoeff( order, k, T )

syms t;
poly=t.^(0:order);
coeff=subs(diff(poly,k),t,T);

end

