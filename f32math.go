package detour

import "math"

var (
	Epsilon32 float32
	NaN       float32
)

func init() {
	Epsilon32 = math.Nextafter32(1, 2) - 1
	NaN = float32(math.NaN())
}

func Abs(v float32) float32 {
	if v < 0 {
		return -v
	}
	return v
}

func Max(v1, v2 float32) float32 {
	if v1 > v2 {
		return v1
	}
	return v2
}

func Approxf32Equal(v1, v2 float32) bool {
	eps := Epsilon32 * 100
	return Abs(v1-v2) < eps*(1.0+Max(Abs(v1), Abs(v2)))
}

/*
   class Approx {
   public:
       explicit Approx ( double value )
       :   m_epsilon( std::numeric_limits<float>::epsilon()*100 ),
           m_scale( 1.0 ),
           m_value( value )
       {}

       Approx( Approx const& other )
       :   m_epsilon( other.m_epsilon ),
           m_scale( other.m_scale ),
           m_value( other.m_value )
       {}

       static Approx custom() {
           return Approx( 0 );
       }

       Approx operator()( double value ) {
           Approx approx( value );
           approx.epsilon( m_epsilon );
           approx.scale( m_scale );
           return approx;
       }

       friend bool operator == ( double lhs, Approx const& rhs ) {
           // Thanks to Richard Harris for his help refining this formula
           return fabs( lhs - rhs.m_value ) < rhs.m_epsilon * (rhs.m_scale + (std::max)( fabs(lhs), fabs(rhs.m_value) ) );
       }

       friend bool operator == ( Approx const& lhs, double rhs ) {
           return operator==( rhs, lhs );
       }

       friend bool operator != ( double lhs, Approx const& rhs ) {
           return !operator==( lhs, rhs );
       }

       friend bool operator != ( Approx const& lhs, double rhs ) {
           return !operator==( rhs, lhs );
       }

       Approx& epsilon( double newEpsilon ) {
           m_epsilon = newEpsilon;
           return *this;
       }

       Approx& scale( double newScale ) {
           m_scale = newScale;
           return *this;
       }

       std::string toString() const {
           std::ostringstream oss;
           oss << "Approx( " << Catch::toString( m_value ) << " )";
           return oss.str();
       }

   private:
       double m_epsilon;
       double m_scale;
       double m_value;
   };
*/
