option explicit
option base 1
option default float
option keyboard repeat 400,25

' 640x480 
mode 2,16

' 320x200
'mode 3,8

dim ORIGIN(3)=(mm.hres/2,mm.vres/2,-500)

' X1,Y1,X2,Y2
dim VIEW_PORT(4)=(100,20,mm.hres-100,mm.vres-20)

const INT_MIN=&H8000000000000000
const INT_MAX=&H7FFFFFFFFFFFFFFF
const PROPORTION=mm.vres/mm.hres
const PROJECTION_PLANE=-400
const HEADER_SIZE=3

const KEY_UP=128,KEY_DOWN=129,KEY_LEFT=130,KEY_RIGHT=131
const KEY_A_LO=97,KEY_A_UP=65
const KEY_R_LO=114,KEY_R_UP=82
const KEY_Z_LO=122,KEY_Z_UP=90


const RED=rgb(red)
const BLUE=rgb(blue)

Main()

sub Main
  local cube(HEADER_SIZE+3*4*4)
  local integer keynum, keypressed
  
  LoadVertices(cube())

  page write 1

  do
    cls
    
    for keynum=1 to keydown(0)
      keypressed=keydown(keynum)
      if keypressed=KEY_UP then
        Translate(cube(), 10,0,1,0)
      elseif keypressed=KEY_DOWN then
        Translate(cube(),-10,0,1,0)
      elseif keypressed=KEY_LEFT then
        Translate(cube(), 10,1,0,0)
      elseif keypressed=KEY_RIGHT then
        Translate(cube(),-10,1,0,0)
      elseif keypressed=KEY_A_LO or keypressed=KEY_A_UP then
        Translate(cube(), 10,0,0,1)
      elseif keypressed=KEY_Z_LO or keypressed=KEY_Z_UP then
        Translate(cube(),-10,0,0,1)
      elseif keypressed=KEY_R_LO or keypressed=KEY_R_UP then
        Rotate(cube(),5,0,1,0)
      endif
      print @(0,300) "KEY: "+str$(keydown(keynum))
    next keynum

    ' Project objects
    ProjectRetangles(cube(),RED)
    ' Draw viewport
    box VIEW_PORT(1),VIEW_PORT(2),VIEW_PORT(3)-VIEW_PORT(1)+1,VIEW_PORT(4)-VIEW_PORT(2)+1,2,BLUE

    ' Print FPS
    print @(0,0) "FPS: "+str$(1000/timer,0,0)
    printv3d(1,cube())
    timer=0

    ' Copy the frame to the screen (page 0)
    page copy 1 to 0
  loop
end sub

sub Scale(vertices() as float, size as float, x as float, y as float, z as float)
  local float v(3), vout(3), sizex, sizey, sizez
  local integer i

  for i=0 to bound(vertices())-1 step 3
    if vertices(i+1)>=posvert(1) then sizex=size*x else sizex=-size*x
    if vertices(i+2)>=posvert(2) then sizey=size*y else sizey=-size*y
    if vertices(i+3)>=posvert(3) then sizez=size*z else sizez=-size*z
    vertices(i+1)=vertices(i+1)+sizex
    vertices(i+2)=vertices(i+2)+sizey
    vertices(i+3)=vertices(i+3)+sizez
  next i
end sub

sub Translate(object(), dist, x, y, z)
  object(1)=object(1)+dist*x
  object(2)=object(2)+dist*y
  object(3)=object(3)+dist*z
end sub

sub Rotate(object(), angle, x, y, z)
  local integer i
  local qrot(5),v(5),vq(5),vout(5)

  math q_create rad(angle),x,y,z,qrot()

  for i=HEADER_SIZE to bound(object())-1 step 3
    v(2)=object(i+1)
    v(3)=object(i+2)
    v(4)=object(i+3)
    math q_rotate qrot(),v(),vout()
    object(i+1)=vout(2)
    object(i+2)=vout(3)
    object(i+3)=vout(4)
  next i
end sub

sub ProjectRetangles(obj(), color as float)
  Project(obj(),4,color)
end sub


sub ProjectTriangles(vertices(), color)
  Project(obj(),3,color)
end sub

sub Project(obj(), numvert as integer, color)
  local integer iproj,idx=1
  local numpols=(bound(obj())-HEADER_SIZE)/3/numvert
  local vx(numvert*numpols),vy(numvert*numpols),n(numpols)
  local vxc(numvert*numpols*2),vyc(numvert*numpols*2)
  local vout(2)

  for iproj=HEADER_SIZE+1 to bound(obj()) step 3
    CalcVertexProjection(obj(),iproj,vout())
    vx(idx)=vout(1)
    vy(idx)=vout(2)
    idx=idx+1
  next
  iproj=ClipPolygons(numvert,vx(),vy(),vxc(),vyc(),n())
  if iproj > 0 then
    local newn(iproj)
    for idx=1 to iproj
      newn(idx)=n(idx)
    next
    polygon newn(),vxc(),vyc(),color
  end if
end sub

function ClipPolygons(numvert as integer,vx(),vy(),vxout(),vyout(),n()) as integer
  local integer ic,c,v,totn=0,tvin,tvout,offset=0
  local vclip1(2),vclip2(2),vcurr(2),vnext(2),vint(2)
  local vxin(bound(vxout())),vyin(bound(vyout()))

  math set -1,vxout()
  math set -1,vyout()
  math set 3,n()

  ' steps trought the polygons
  for ic=1 to bound(vx()) step numvert
    for v=1 to numvert
      c=ic+v-1
      vxout(v+offset)=vx(c):vyout(v+offset)=vy(c)
    next v
    tvout=numvert

    ' steps trought the clipping edges
    for c=1 to 4
      if c=1 then
        vclip1(1)=INT_MIN:vclip1(2)=VIEW_PORT(2)
        vclip2(1)=INT_MAX:vclip2(2)=VIEW_PORT(2)
      elseif c=2 then
        vclip1(1)=VIEW_PORT(3):vclip1(2)=INT_MIN
        vclip2(1)=VIEW_PORT(3):vclip2(2)=INT_MAX
      elseif c=3 then
        vclip1(1)=INT_MIN:vclip1(2)=VIEW_PORT(4)
        vclip2(1)=INT_MAX:vclip2(2)=VIEW_PORT(4)
      else
        vclip1(1)=VIEW_PORT(1):vclip1(2)=INT_MIN
        vclip2(1)=VIEW_PORT(1):vclip2(2)=INT_MAX
      end if

      ' Initializes vxin and vyin
      math scale vxout(),1,vxin()
      math scale vyout(),1,vyin()
      ' Initializes total vertices counter
      tvin=tvout:tvout=0

      ' steps trought the vertices
      for v=1+offset to tvin+offset
        vcurr(1)=vxin(v):vcurr(2)=vyin(v)
        if v=tvin+offset then
          vnext(1)=vxin(offset+1):vnext(2)=vyin(offset+1)
        else
          vnext(1)=vxin(v+1):vnext(2)=vyin(v+1)
        end if

        if IsInside(vcurr(),vclip1(),vclip2()) then
          if IsInside(vnext(),vclip1(),vclip2()) then
            tvout=tvout+1        
            vxout(tvout+offset)=vnext(1):vyout(tvout+offset)=vnext(2)
          else
            CalcIntersection(vcurr(),vnext(),vclip1(),vclip2(),vint())
            tvout=tvout+1        
            vxout(tvout+offset)=vint(1):vyout(tvout+offset)=vint(2)
          end if
        elseif IsInside(vnext(),vclip1(),vclip2()) then
          CalcIntersection(vcurr(),vnext(),vclip1(),vclip2(),vint())
          tvout=tvout+1
          vxout(tvout+offset)=vint(1):vyout(tvout+offset)=vint(2)
          tvout=tvout+1
          vxout(tvout+offset)=vnext(1):vyout(tvout+offset)=vnext(2)
        end if
      next v
    next c
    if tvout > 2 then
      offset=offset+tvout
      totn=totn+1
      n(totn)=tvout
    end if
  next ic
  ClipPolygons=totn
end function

function prt(v) as string
  prt=str$(v,0,0)
  if v=INT_MIN then
    prt="-I"
  elseif v=INT_MAX then
    prt=" I"
  end if
end function

sub CalcIntersection(v1(),v2(),c1(),c2(),vout())
  local s1(2)=(v2(1)-v1(1),v2(2)-v1(2))
  local s2(2)=(c2(1)-c1(1),c2(2)-c1(2))
  local d=-s2(1)*s1(2)+s1(1)*s2(2)
  local s=(-s1(2)*(v1(1)-c1(1))+s1(1)*(v1(2)-c1(2)))/d
  local t=( s2(1)*(v1(2)-c1(2))-s2(2)*(v1(1)-c1(1)))/d

  if s>=0 and s<=1 and t>=0 and t<=1 then
    vout(1)=t*s1(1)+v1(1)
    vout(2)=t*s1(2)+v1(2)
  end if
end sub

function IsInside(v(),c1(),c2()) as integer
  IsInside=0
  if c1(1)=c2(1) then ' Same X
    if c1(1)=VIEW_PORT(1) then IsInside=v(1)>=c1(1) else IsInside=v(1)<=c1(1)
  elseif c1(2)=c2(2) then ' Same Y
    if c1(2)=VIEW_PORT(2) then IsInside=v(2)>=c1(2) else IsInside=v(2)<=c1(2)
  end if
end function

' Calculates the 3D vertex projection in a 2D space using right-handed system
sub CalcVertexProjection(obj(), vidx as integer, vout())
  local float zproj=obj(vidx+2)+obj(3)-ORIGIN(3)
  if zproj=0 then zproj=0.1
  vout(1)=(obj(vidx)+obj(1))*PROPORTION*PROJECTION_PLANE/zproj+ORIGIN(1)
  vout(2)=(obj(vidx+1)+obj(2))*PROJECTION_PLANE/zproj+ORIGIN(2)
end sub

sub LoadVertices(vertices())
  local integer i
  for i=HEADER_SIZE+1 to bound(vertices())
    read vertices(i)
  next i
end sub

sub print3d(printline,x,y,z)
  print @(0,printline * 20) str$(x,0,1)+","+str$(y,0,1)+","+str$(z,0,1)
end sub

sub printv3d(printline,vertices())
  print @(0,printline * 20) str$(vertices(1),0,1)+","+str$(vertices(2),0,1)+","+str$(vertices(3),0,1)
end sub

sub printv2d(printline,vertices())
  print @(0,printline * 20) str$(vertices(1),0,1)+","+str$(vertices(2),0,1)
end sub

' Cube points
data -100,-100,100,  100,-100,100,   100,100,100,   -100,100,100
data  100,-100,100,  100,-100,-100,  100,100,-100,  100,100,100
data  100,-100,-100, -100,-100,-100, -100,100,-100, 100,100,-100
data -100,-100,-100, -100,-100,100,  -100,100,100, -100,100,-100


