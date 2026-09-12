#pragma once
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cassert>
#define SH110X_WHITE 1
#define SH110X_BLACK 0
class Adafruit_SH1106G {
public:
 int frames=0,x=0,y=0,size=1,color=1;
 std::vector<std::string> texts;
 std::ostringstream svg;
 void clearDisplay(){texts.clear();svg.str("");svg.clear();}
 void setTextWrap(bool){}
 void setTextColor(int c){color=c;}
 void setTextSize(int s){size=s;}
 void setCursor(int xx,int yy){x=xx;y=yy;}
 const char* ink(int c){return c?"white":"black";}
 void print(const char* s){
   assert(x>=0 && y>=0 && x+int(std::string(s).size())*6*size<=128 && y+8*size<=64);
   texts.push_back(s);
   svg<<"<text x='"<<x<<"' y='"<<y+7*size<<"' fill='"<<ink(color)<<"' font-family='monospace' font-size='"<<8*size<<"' textLength='"<<std::string(s).size()*6*size<<"' lengthAdjust='spacingAndGlyphs'>"<<s<<"</text>";
 }
 void drawRect(int x,int y,int w,int h,int c){rect(x,y,w,h,c,false);}
 void fillRect(int x,int y,int w,int h,int c){rect(x,y,w,h,c,true);}
 void rect(int x,int y,int w,int h,int c,bool fill){svg<<"<rect x='"<<x<<"' y='"<<y<<"' width='"<<w<<"' height='"<<h<<"' fill='"<<(fill?ink(c):"none")<<"' stroke='"<<ink(c)<<"' stroke-width='.6'/>";}
 void drawPixel(int x,int y,int c){fillRect(x,y,1,1,c);}
 void drawLine(int x,int y,int xx,int yy,int c){svg<<"<path d='M"<<x<<","<<y<<" L"<<xx<<","<<yy<<"' stroke='"<<ink(c)<<"' fill='none' stroke-width='1'/>";}
 void drawFastVLine(int x,int y,int h,int c){drawLine(x,y,x,y+h-1,c);}
 void drawFastHLine(int x,int y,int w,int c){drawLine(x,y,x+w-1,y,c);}
 void fillTriangle(int x,int y,int xx,int yy,int xxx,int yyy,int c){svg<<"<path d='M"<<x<<","<<y<<" L"<<xx<<","<<yy<<" L"<<xxx<<","<<yyy<<" Z' fill='"<<ink(c)<<"'/>";}
 void display(){++frames;}
 bool has(const std::string& text){for(auto&s:texts)if(s==text)return true;return false;}
 void save(const std::string& path){std::ofstream f(path);f<<"<svg xmlns='http://www.w3.org/2000/svg' width='512' height='256' viewBox='0 0 128 64'><rect width='128' height='64' fill='black'/>"<<svg.str()<<"</svg>";}
};
