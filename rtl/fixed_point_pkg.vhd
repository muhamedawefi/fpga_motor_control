library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package fixed_point_pkg is
    
    -- Fixed-point multiply with rounding
    function fp_mult(
        a : signed;
        b : signed;
        a_frac : integer;
        b_frac : integer;
        out_frac : integer
    ) return signed;
    
    -- Saturation
    function saturate(
        value : signed;
        min_val : signed;
        max_val : signed
    ) return signed;
    
end package fixed_point_pkg;

package body fixed_point_pkg is
    

function fp_mult(
    a : signed;
    b : signed;
    a_frac : integer;
    b_frac : integer;
    out_frac : integer
) return signed is
    variable product : signed(a'length + b'length - 1 downto 0);
    variable shift   : integer;
    variable temp    : signed(product'length - 1 downto 0);
begin
    product := a * b;
    shift := a_frac + b_frac - out_frac;

    if shift > 0 then
        temp := shift_right(
            product + shift_left(to_signed(1, product'length), shift-1),
            shift
        );
    elsif shift < 0 then
        temp := shift_left(product, -shift);
    else
        temp := product;
    end if;

    return temp; -- we'll refine later if needed
end function;
--NEVER shrink precision inside math functions
--ONLY shrink at system boundaries
    
    function saturate(
        value : signed;
        min_val : signed;
        max_val : signed
    ) return signed is
    begin
        if value > max_val then
            return max_val;
        elsif value < min_val then
            return min_val;
        else
            return value;
        end if;
    end function;
    
end package body fixed_point_pkg;