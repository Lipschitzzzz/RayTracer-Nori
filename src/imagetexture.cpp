#include <nori/texture.h>
#include <nori/bitmap.h>

NORI_NAMESPACE_BEGIN
class ImageTexture : public Texture
{
private:
    Bitmap* bmp;
    std::string fileName;
    int textureWidth;
    int textureHeight;
public:

    ImageTexture(const PropertyList& props)
    {

        fileName = props.getString("fileName", " ");
        if (fileName != "")
        {
            // loadTextureFile(fileName);
        }
        
    }
    EClassType getClassType() const { return ETexture; }

    void loadTextureFile(const std::string textureName)
    {
        bmp = new Bitmap(textureName);
        textureWidth = bmp->cols();
        textureHeight = bmp->rows();
        if (bmp == nullptr)
        {
            std::cout << "open texture file failed" << std::endl;
        }
        std::cout << "load texture successfully, width " << textureWidth << " height " << textureHeight << std::endl;
    }

    Color3f eval(const Point2f& uv) const
    {

        if (fileName != "")
        {
            int u = uv.x() * textureWidth;
            int v = uv.y() * textureHeight;

            return bmp->coeff(u, v);
        }
        return Color3f(1.0f);
    }

    std::string toString() const
    {
        return tfm::format(
            "ImageTexture[\n"
            "  fileName = %s,\n"
            "]",
            fileName
        );
    }


};

NORI_REGISTER_CLASS(ImageTexture, "imagetexture")
NORI_NAMESPACE_END